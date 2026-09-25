#include <visualization/webrtc_stream.hpp>

// GStreamer headers for WebRTC streaming
#include <gst/app/gstappsrc.h>
#include <gst/sdp/gstsdpmessage.h>
#include <gst/webrtc/webrtc.h>

// Boost.Beast for HTTP + WebSocket signaling (header-only, single port)
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/write.hpp>

// nlohmann/json for signaling message handling
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <memory>
#include <cstdint>
#include <iostream>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
using json = nlohmann::json;

namespace visualization
{
    namespace
    {
        constexpr const char kBrowserHtml[] = R"HTML(
            <!doctype html>
                <html>
                    <head>
                        <meta charset="utf-8">
                        <meta name="viewport" content="width=device-width,initial-scale=1">
                        <title>VisionPilot</title>
                        <style>
                            html,body{
                                height:100%;
                                margin:0;
                                background:#000
                            }
                            video{
                                width:100%;
                                height:100%;
                                object-fit:contain;
                                background:#000;
                                display:block
                            }
                        </style>
                    </head>
                    <body>
                        <video id="video" autoplay playsinline muted></video>
                        <script>
                            const video=document.getElementById('video');
                            const pendingCandidates=[];
                            const pc=new RTCPeerConnection();
                            pc.ontrack=e=>{video.srcObject=e.streams[0]};
                            pc.onicecandidate=e=>{
                                if(e.candidate)ws&&ws.readyState===WebSocket.OPEN&&ws.send(
                                    JSON.stringify({
                                        type:'candidate',
                                        candidate:e.candidate.candidate,
                                        sdpMLineIndex:e.candidate.sdpMLineIndex
                                    })
                                )
                            };
                            const scheme=location.protocol==='https:'?'wss://':'ws://';
                            const ws=new WebSocket(scheme+location.host+'/'+'ws');
                            async function drainPendingCandidates(){
                                if(!pc.remoteDescription){
                                    return;
                                }
                                while(pendingCandidates.length>0){
                                    const c=pendingCandidates.shift();
                                    await pc.addIceCandidate(c);
                                }
                            }
                            ws.onmessage=async ev=>{
                                const p=JSON.parse(ev.data);
                                if(p.type==='offer'){
                                    await pc.setRemoteDescription({
                                        type:'offer',
                                        sdp:p.sdp
                                    });
                                    await drainPendingCandidates();
                                    const a=await pc.createAnswer();
                                    await pc.setLocalDescription(a);
                                    ws.send(JSON.stringify({
                                        type:'answer',
                                        sdp:pc.localDescription.sdp
                                    }));
                                } else if(p.type==='candidate') {
                                    const candidate={
                                        candidate:p.candidate,
                                        sdpMLineIndex:p.sdpMLineIndex
                                    };
                                    if(!pc.remoteDescription){
                                        pendingCandidates.push(candidate);
                                        return;
                                    }
                                    try {
                                        await pc.addIceCandidate(candidate);
                                    } catch(e) {
                                        console.error('Error adding ICE candidate:', e);
                                    }
                                }
                            };
                        </script>
                    </body>
                </html>
        )HTML";


        // Plain-HTTP viewer for the same rendered frame. WebRTC media cannot
        // traverse an HTTP-only tunnel (cloudflared forwards TCP/HTTP, not the
        // UDP ICE path), so the MJPEG endpoint below mirrors the visualization
        // frame as a browser-friendly multipart stream.
        constexpr const char kMjpegHtml[] = R"HTML(
            <!doctype html>
                <html>
                    <head>
                        <meta charset="utf-8">
                        <meta name="viewport" content="width=device-width,initial-scale=1">
                        <title>VisionPilot MJPEG</title>
                        <style>
                            html,body{
                                height:100%;
                                margin:0;
                                background:#000
                            }
                            img{
                                width:100%;
                                height:100%;
                                object-fit:contain;
                                background:#000;
                                display:block
                            }
                        </style>
                    </head>
                    <body>
                        <img src="/mjpeg" alt="VisionPilot">
                    </body>
                </html>
        )HTML";


        // Helper func to initialize GStreamer once (thread-safe)
        void init_gstreamer_once()
        {
            static std::once_flag once;
            std::call_once(once, []()
            {
                gst_init(nullptr, nullptr);
            });
        }


        // Helper func to generate JSON signaling message for SDP offer
        std::string make_offer_message(
            const std::string& sdp_offer
        )
        {
            json msg = {{"type", "offer"}, {"sdp", sdp_offer}};
            return msg.dump();
        };


        // Helper func to generate JSON signaling message for ICE candidate
        std::string make_candidate_message(
            int sdp_mline_index,
            const std::string& candidate
        )
        {
            json msg = {
                {"type", "candidate"},
                {"sdpMLineIndex", sdp_mline_index},
                {"candidate", candidate}
            };
            return msg.dump();
        };


        // Helper func to ensure all received frames are in BGR format for WebRTC streaming
        cv::Mat ensure_bgr_frame(
            const cv::Mat& frame
        )
        {
            if (frame.empty())
            {
                return frame;
            };

            if (frame.type() == CV_8UC3)
            {
                return frame.isContinuous() ? frame : frame.clone();
            };

            cv::Mat converted;
            if (frame.type() == CV_8UC1)
            {
                cv::cvtColor(
                    frame,
                    converted,
                    cv::COLOR_GRAY2BGR
                );
            }
            else if (frame.type() == CV_8UC4)
            {
                cv::cvtColor(
                    frame,
                    converted,
                    cv::COLOR_BGRA2BGR
                );
            }
            else
            {
                frame.convertTo(
                    converted,
                    CV_8UC3
                );
                if (converted.channels() != 3)
                {
                    return cv::Mat();
                };
            };

            return converted;
        };
    }; // namespace

    struct WebRTCStreamer::Impl
    {
        explicit Impl(Config config_in) : config(std::move(config_in))
        {
        };

        bool start();
        bool stop();
        bool push_frame(const cv::Mat& frame);
        bool has_client() const;
        void queue_signal(const std::string& signal);
        void flush_pending_signals();
        void queue_remote_candidate(
            int sdp_mline_index,
            const std::string& candidate
        );
        void flush_pending_remote_candidates();
        void handle_signaling_message(const std::string& msg);

        void accept_loop();
        void handle_connection(tcp::socket socket);
        void handle_websocket_session(
            tcp::socket socket,
            http::request<http::string_body> req
        );
        void handle_mjpeg_connection(tcp::socket socket);
        void encode_mjpeg_frame(const cv::Mat& bgr_frame);


        // Config for WebRTC streaming
        Config config;

        // Boost.Asio/Beast networking state (io_context owns the acceptor; no
        // separate compiled signaling library, so no ABI surface to mismatch)
        net::io_context ioc{1};
        std::unique_ptr<tcp::acceptor> acceptor;
        std::thread accept_thread;
        std::atomic<bool> server_running{false};


        // GStreamer elements for WebRTC streaming
        GstElement* pipeline = nullptr;
        GstElement* appsrc = nullptr;
        GstElement* webrtc = nullptr;


        // State management for signaling and streaming
        mutable std::mutex signal_mutex;
        std::mutex write_mutex; // serializes writes to client_ws separately from pointer swaps
        std::shared_ptr<websocket::stream<tcp::socket>> client_ws;
        std::vector<std::string> pending_signals;


        // State management for remote ICE candidates received before remote description is set
        std::mutex remote_candidate_mutex;
        std::vector<std::pair<int, std::string>> pending_remote_candidates;
        std::atomic<bool> remote_description_ready{false};


        // State management for streaming
        std::atomic<bool> running{false};
        std::atomic<uint64_t> frame_index{0};
        bool caps_configured = false;
        int configured_width = 0;
        int configured_height = 0;


        // Plain-HTTP MJPEG mirror of the rendered frame. The JPEG is encoded
        // only while at least one /mjpeg client is connected, at ~10 fps.
        std::mutex mjpeg_mutex;
        std::vector<uchar> mjpeg_jpeg;
        std::atomic<uint64_t> mjpeg_sequence{0};
        std::atomic<int> mjpeg_clients{0};
        std::chrono::steady_clock::time_point mjpeg_last_encode;
        bool mjpeg_encode_seen = false;
    };


    // Websocket handler for remote candidate
    void handle_remote_candidate(
        WebRTCStreamer::Impl* impl,
        int sdp_mline_index,
        const std::string& candidate
    )
    {
        g_signal_emit_by_name(
            impl->webrtc,
            "add-ice-candidate",
            static_cast<guint>(sdp_mline_index),
            candidate.c_str()
        );
    };


    // Websocket handler for remote description (SDP answer)
    void handle_remote_description(
        WebRTCStreamer::Impl* impl,
        const std::string& sdp_text
    )
    {
        GstSDPMessage* sdp = nullptr;
        if (
            (gst_sdp_message_new_from_text(sdp_text.c_str(), &sdp) != GST_SDP_OK) ||
            (sdp == nullptr)
        )
        {
            return;
        }

        GstWebRTCSessionDescription* answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
        g_signal_emit_by_name(
            impl->webrtc,
            "set-remote-description",
            answer,
            nullptr
        );
        gst_webrtc_session_description_free(answer);

        impl->remote_description_ready.store(true, std::memory_order_release);
        impl->flush_pending_remote_candidates();
    };


    // Callback for when SDP offer is created to set local description and send offer to client
    void on_offer_created(
        GstPromise* promise,
        gpointer user_data
    )
    {
        auto* impl = static_cast<WebRTCStreamer::Impl*>(user_data);
        const GstStructure* reply = gst_promise_get_reply(promise);
        GstWebRTCSessionDescription* offer = nullptr;

        if (reply != nullptr)
        {
            gst_structure_get(
                reply,
                "offer",
                GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer,
                NULL
            );
        }

        if (offer == nullptr)
        {
            gst_promise_unref(promise);
            return;
        }

        g_signal_emit_by_name(
            impl->webrtc,
            "set-local-description",
            offer,
            nullptr
        );

        gchar* sdp_text = gst_sdp_message_as_text(offer->sdp);
        if (sdp_text != nullptr)
        {
            impl->queue_signal(make_offer_message(sdp_text));
            g_free(sdp_text);
        }

        gst_webrtc_session_description_free(offer);
        gst_promise_unref(promise);
    };


    void on_negotiation_needed(
        GstElement* element,
        gpointer user_data
    )
    {
        (void)element;

        auto* impl = static_cast<WebRTCStreamer::Impl*>(user_data);
        GstPromise* promise = gst_promise_new_with_change_func(
            on_offer_created,
            impl,
            nullptr
        );

        g_signal_emit_by_name(
            impl->webrtc,
            "create-offer",
            nullptr,
            promise
        );
    };


    void on_ice_candidate(
        GstElement* element,
        guint mline_index,
        gchar* candidate,
        gpointer user_data
    )
    {
        (void)element;
        auto* impl = static_cast<WebRTCStreamer::Impl*>(user_data);

        if (candidate != nullptr)
        {
            impl->queue_signal(make_candidate_message(
                static_cast<int>(mline_index),
                candidate
            ));
        };
    };


    // Dispatch an incoming signaling message (SDP answer or ICE candidate) to GStreamer
    void WebRTCStreamer::Impl::handle_signaling_message(const std::string& msg)
    {
        json j = json::parse(msg, nullptr, false);
        if (j.is_discarded() || !j.contains("type"))
        {
            return;
        }

        const std::string signal_type = j["type"].get<std::string>();

        if (signal_type == "answer" && j.contains("sdp"))
        {
            handle_remote_description(this, j["sdp"].get<std::string>());
        }
        else if (
            signal_type == "candidate" &&
            j.contains("candidate") &&
            j.contains("sdpMLineIndex")
        )
        {
            const int sdp_mline_index = j["sdpMLineIndex"].get<int>();
            const std::string candidate = j["candidate"].get<std::string>();

            if (remote_description_ready.load(std::memory_order_acquire))
            {
                handle_remote_candidate(this, sdp_mline_index, candidate);
            }
            else
            {
                queue_remote_candidate(sdp_mline_index, candidate);
            }
        }
    };

    void WebRTCStreamer::Impl::accept_loop()
    {
        while (server_running.load(std::memory_order_acquire))
        {
            tcp::socket socket(ioc);
            boost::system::error_code ec;

            acceptor->non_blocking(true);
            acceptor->accept(socket, ec);

            if (ec == boost::asio::error::would_block)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            if (ec)
            {
                if (server_running.load(std::memory_order_acquire))
                {
                    g_printerr("[WebRTCStreamer] accept error: %s\n", ec.message().c_str());
                }
                continue;
            }

            std::thread(&WebRTCStreamer::Impl::handle_connection, this, std::move(socket)).detach();
        }
    }


    // Reads the initial HTTP request off a freshly accepted socket and dispatches
    // to either the plain-HTML response or the WebSocket upgrade path
    void WebRTCStreamer::Impl::handle_connection(tcp::socket socket)
    {
        try
        {
            beast::flat_buffer buffer;
            http::request<http::string_body> req;

            boost::system::error_code ec;
            http::read(socket, buffer, req, ec);
            if (ec)
            {
                return;
            }

            if (websocket::is_upgrade(req))
            {
                handle_websocket_session(std::move(socket), std::move(req));
                return;
            }

            if (req.target() == "/mjpeg")
            {
                handle_mjpeg_connection(std::move(socket));
                return;
            }

            http::response<http::string_body> res{http::status::ok, req.version()};
            res.set(http::field::server, "VisionPilot");
            res.set(http::field::content_type, "text/html");
            res.keep_alive(false);
            res.body() = (req.target() == "/view") ? kMjpegHtml : kBrowserHtml;
            res.prepare_payload();

            http::write(socket, res, ec);
            socket.shutdown(tcp::socket::shutdown_send, ec);
        }
        catch (const std::exception& e)
        {
            g_printerr("[WebRTCStreamer] connection error: %s\n", e.what());
        }
    };


    // Upgrades the connection to a WebSocket, registers it as the active client,
    // and blocks reading signaling messages until the client disconnects
    void WebRTCStreamer::Impl::handle_websocket_session(
        tcp::socket socket,
        http::request<http::string_body> req
    )
    {
        auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));

        try
        {
            ws->accept(req);
        }
        catch (const std::exception& e)
        {
            g_printerr("[WebRTCStreamer] websocket accept failed: %s\n", e.what());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            client_ws = ws;
        }

        flush_pending_signals();

        beast::flat_buffer read_buffer;

        while (server_running.load(std::memory_order_acquire))
        {
            boost::system::error_code ec;
            ws->read(read_buffer, ec);

            if (ec == websocket::error::closed || ec)
            {
                break;
            }

            if (ws->got_text())
            {
                std::string msg = beast::buffers_to_string(read_buffer.data());
                handle_signaling_message(msg);
            }

            read_buffer.consume(read_buffer.size());
        }

        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            if (client_ws == ws)
            {
                client_ws.reset();
            }
        }
    };


    // JPEG-encodes the rendered frame for the /mjpeg viewer, at most ~10 fps
    // and only while a viewer is connected. Runs on the VisionPilot main loop
    // thread (called from push_frame), like the existing GStreamer push.
    void WebRTCStreamer::Impl::encode_mjpeg_frame(const cv::Mat& bgr_frame)
    {
        if (mjpeg_clients.load(std::memory_order_acquire) <= 0)
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        {
            std::lock_guard<std::mutex> lock(mjpeg_mutex);
            if (
                mjpeg_encode_seen &&
                std::chrono::duration_cast<std::chrono::milliseconds>(now - mjpeg_last_encode).count() < 90
            )
            {
                return;
            }
            mjpeg_last_encode = now;
            mjpeg_encode_seen = true;
        }

        std::vector<uchar> jpeg;
        const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", bgr_frame, jpeg, params))
        {
            return;
        }

        std::lock_guard<std::mutex> lock(mjpeg_mutex);
        mjpeg_jpeg.swap(jpeg);
        mjpeg_sequence.fetch_add(1, std::memory_order_release);
    }


    // Streams the latest JPEG as multipart/x-mixed-replace until the client
    // disconnects. The response has no Content-Length: the connection close
    // delimits the stream.
    void WebRTCStreamer::Impl::handle_mjpeg_connection(tcp::socket socket)
    {
        struct ClientCounter
        {
            std::atomic<int>* counter;
            ~ClientCounter()
            {
                counter->fetch_sub(1, std::memory_order_acq_rel);
            }
        } client_counter{&mjpeg_clients};

        mjpeg_clients.fetch_add(1, std::memory_order_acq_rel);

        try
        {
            const std::string response_header =
                "HTTP/1.1 200 OK\r\n"
                "Server: VisionPilot\r\n"
                "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                "Cache-Control: no-store\r\n"
                "Pragma: no-cache\r\n"
                "Connection: close\r\n"
                "\r\n";

            boost::system::error_code ec;
            net::write(socket, net::buffer(response_header), ec);
            if (ec)
            {
                return;
            }

            uint64_t last_sent = 0;
            while (server_running.load(std::memory_order_acquire))
            {
                std::vector<uchar> frame;
                uint64_t sequence = 0;
                {
                    std::lock_guard<std::mutex> lock(mjpeg_mutex);
                    frame = mjpeg_jpeg;
                    sequence = mjpeg_sequence.load(std::memory_order_acquire);
                }

                if (frame.empty() || sequence == last_sent)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                    continue;
                }
                last_sent = sequence;

                const std::string part_header =
                    "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                    std::to_string(frame.size()) + "\r\n\r\n";
                net::write(socket, net::buffer(part_header), ec);
                if (ec)
                {
                    break;
                }
                net::write(socket, net::buffer(frame), ec);
                if (ec)
                {
                    break;
                }
                net::write(socket, net::buffer("\r\n", 2), ec);
                if (ec)
                {
                    break;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }

            socket.shutdown(tcp::socket::shutdown_send, ec);
        }
        catch (const std::exception& e)
        {
            g_printerr("[WebRTCStreamer] mjpeg connection error: %s\n", e.what());
        }
    };


    // Full implementation of WebRTCStreamer
    bool WebRTCStreamer::Impl::start()
    {
        // 1. Init GStreamer (thread-safe)
        init_gstreamer_once();

        // 2. Init Boost.Asio acceptor for combined HTTP + WebSocket signaling

        try
        {
            acceptor = std::make_unique<tcp::acceptor>(
                ioc,
                tcp::endpoint(net::ip::make_address(config.host), config.port)
            );
        }
        catch (const std::exception& e)
        {
            g_printerr("[WebRTCStreamer] failed to bind port %d: %s\n", config.port, e.what());
            return false;
        }

        server_running.store(true, std::memory_order_release);
        accept_thread = std::thread(&WebRTCStreamer::Impl::accept_loop, this);

        g_printerr("[WebRTCStreamer] listening on port %d\n", config.port);
        g_printerr(
            "[WebRTCStreamer] MJPEG viewer: http://%s:%d/view (stream: /mjpeg)\n",
            config.host.c_str(), config.port
        );

        // 3. Build GStreamer pipeline

        GError* pipeline_error = nullptr;
        pipeline = gst_parse_launch(
            "appsrc name=source is-live=true format=time do-timestamp=true block=true ! "
            "queue leaky=downstream max-size-buffers=2 ! videoconvert ! "
            "vp8enc target-bitrate=4000000 cpu-used=4 deadline=1 "
            "keyframe-max-dist=30 end-usage=cbr ! "
            "rtpvp8pay pt=96 ! application/x-rtp,media=video,encoding-name=VP8,payload=96,clock-rate=90000 ! "
            "webrtcbin name=webrtc bundle-policy=max-bundle",
            &pipeline_error
        );

        if (pipeline == nullptr)
        {
            if (pipeline_error != nullptr)
            {
                g_printerr("[WebRTCStreamer] gst_parse_launch failed: %s\n", pipeline_error->message);
                g_error_free(pipeline_error);
            }
            else
            {
                g_printerr("[WebRTCStreamer] gst_parse_launch failed (no error provided)\n");
            }
            stop();
            return false;
        }

        appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");
        webrtc = gst_bin_get_by_name(GST_BIN(pipeline), "webrtc");
        g_printerr("[WebRTCStreamer] pipeline created, appsrc=%p webrtc=%p\n", (void*)appsrc, (void*)webrtc);

        if (
            (appsrc == nullptr) ||
            (webrtc == nullptr)
        )
        {
            stop();
            return false;
        }

        g_object_set(G_OBJECT(appsrc), "block", TRUE, nullptr);
        g_signal_connect(webrtc, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), this);
        g_signal_connect(webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate), this);

        // 4. Set pipeline to PLAYING state

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        {
            g_printerr("[WebRTCStreamer] gst_element_set_state PLAYING failed\n");
            stop();
            return false;
        }

        g_printerr("[WebRTCStreamer] pipeline set to PLAYING\n");

        running.store(true, std::memory_order_release);

        return true;
    };


    bool WebRTCStreamer::Impl::stop()
    {
        running.store(false, std::memory_order_release);
        server_running.store(false, std::memory_order_release);

        if (pipeline != nullptr)
        {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }

        // Unblock the accept loop by closing the acceptor
        if (acceptor != nullptr)
        {
            boost::system::error_code ec;
            acceptor->cancel(ec);
            acceptor->close(ec);
        }

        if (accept_thread.joinable())
        {
            accept_thread.join();
        }

        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            client_ws.reset();
            // detached session threads own their own shared_ptr copy and will exit on next read error
        }

        if (appsrc != nullptr)
        {
            gst_object_unref(appsrc);
            appsrc = nullptr;
        }

        if (webrtc != nullptr)
        {
            gst_object_unref(webrtc);
            webrtc = nullptr;
        }

        if (pipeline != nullptr)
        {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }

        acceptor.reset();

        return true;
    };


    bool WebRTCStreamer::Impl::push_frame(
        const cv::Mat& frame
    )
    {
        if (
            (!running.load(std::memory_order_acquire)) ||
            (appsrc == nullptr) ||
            (frame.empty())
        )
        {
            return false;
        }

        cv::Mat bgr_frame = ensure_bgr_frame(frame);
        if (bgr_frame.empty())
        {
            return false;
        }

        encode_mjpeg_frame(bgr_frame);

        if (
            (!caps_configured) ||
            (bgr_frame.cols != configured_width) ||
            (bgr_frame.rows != configured_height)
        )
        {
            GstCaps* caps = gst_caps_new_simple(
                "video/x-raw",
                "format", G_TYPE_STRING, "BGR",
                "width", G_TYPE_INT, bgr_frame.cols,
                "height", G_TYPE_INT, bgr_frame.rows,
                nullptr
            );

            if (config.frame_rate > 0.0)
            {
                const int fps_numerator = std::max(1, static_cast<int>(std::lround(config.frame_rate)));
                gst_caps_set_simple(
                    caps,
                    "framerate",
                    GST_TYPE_FRACTION,
                    fps_numerator,
                    1,
                    nullptr
                );
            }

            gst_app_src_set_caps(GST_APP_SRC(appsrc), caps);
            gst_caps_unref(caps);

            caps_configured = true;
            configured_width = bgr_frame.cols;
            configured_height = bgr_frame.rows;
        }

        const std::size_t payload_size = static_cast<std::size_t>(bgr_frame.total() * bgr_frame.elemSize());
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, payload_size, nullptr);
        if (buffer == nullptr)
        {
            return false;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
        {
            gst_buffer_unref(buffer);
            return false;
        }

        std::memcpy(
            map.data,
            bgr_frame.data,
            payload_size
        );
        gst_buffer_unmap(buffer, &map);

        const guint64 duration_ns = config.frame_rate > 0.0
                                        ? static_cast<guint64>(GST_SECOND / config.frame_rate)
                                        : GST_CLOCK_TIME_NONE;
        const guint64 pts_ns = (
            duration_ns == GST_CLOCK_TIME_NONE ? 0 : frame_index.fetch_add(1, std::memory_order_acq_rel) * duration_ns
        );
        GST_BUFFER_PTS(buffer) = pts_ns;
        GST_BUFFER_DTS(buffer) = pts_ns;
        GST_BUFFER_DURATION(buffer) = duration_ns;

        return (gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer) == GST_FLOW_OK);
    };


    bool WebRTCStreamer::Impl::has_client() const
    {
        std::lock_guard<std::mutex> lock(signal_mutex);
        return client_ws != nullptr && client_ws->is_open();
    };


    void WebRTCStreamer::Impl::queue_signal(
        const std::string& signal
    )
    {
        std::shared_ptr<websocket::stream<tcp::socket>> ws;
        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            if (client_ws == nullptr || !client_ws->is_open())
            {
                pending_signals.push_back(signal);
                return;
            }
            ws = client_ws;
        }

        std::lock_guard<std::mutex> wlock(write_mutex);
        try
        {
            ws->text(true);
            ws->write(net::buffer(signal));
        }
        catch (const std::exception& e)
        {
            g_printerr("[WebRTCStreamer] websocket write failed: %s\n", e.what());
        }
    }


    void WebRTCStreamer::Impl::flush_pending_signals()
    {
        std::vector<std::string> queued_signals;
        std::shared_ptr<websocket::stream<tcp::socket>> ws;

        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            if (
                (client_ws == nullptr) ||
                (!client_ws->is_open()) ||
                (pending_signals.empty())
            )
            {
                return;
            }

            queued_signals.swap(pending_signals);
            ws = client_ws;
        }

        std::lock_guard<std::mutex> wlock(write_mutex);
        for (const auto& signal : queued_signals)
        {
            try
            {
                ws->text(true);
                ws->write(net::buffer(signal));
            }
            catch (const std::exception& e)
            {
                g_printerr("[WebRTCStreamer] websocket write failed: %s\n", e.what());
                break;
            }
        }
    };


    void WebRTCStreamer::Impl::queue_remote_candidate(
        int sdp_mline_index,
        const std::string& candidate
    )
    {
        std::lock_guard<std::mutex> lock(remote_candidate_mutex);
        pending_remote_candidates.emplace_back(
            sdp_mline_index,
            candidate
        );
    };


    void WebRTCStreamer::Impl::flush_pending_remote_candidates()
    {
        std::vector<std::pair<int, std::string>> queued_candidates;

        {
            std::lock_guard<std::mutex> lock(remote_candidate_mutex);
            queued_candidates.swap(pending_remote_candidates);
        }

        for (const auto& candidate : queued_candidates)
        {
            handle_remote_candidate(
                this,
                candidate.first,
                candidate.second
            );
        }
    };


    // WebRTCStreamer constructor and destructor and aux. funcs (unchanged)

    WebRTCStreamer::WebRTCStreamer(Config config) : impl(std::make_unique<Impl>(std::move(config)))
    {
    }

    WebRTCStreamer::WebRTCStreamer() : WebRTCStreamer(Config())
    {
    }

    bool WebRTCStreamer::init(Config config)
    {
        if (impl != nullptr)
        {
            stop();
        }

        impl = std::make_unique<Impl>(std::move(config));
        std::cout << "Open browser at: " << browser_url() << "\n";

        return start();
    }

    bool WebRTCStreamer::init(uint16_t port)
    {
        Config config;
        config.port = port;
        return init(std::move(config));
    }

    WebRTCStreamer::~WebRTCStreamer()
    {
        WebRTCStreamer::stop();
    }

    bool WebRTCStreamer::start() const
    {
        return impl != nullptr && impl->start();
    }

    bool WebRTCStreamer::stop()
    {
        if (impl != nullptr)
        {
            return impl->stop();
        }
        return true;
    };

    bool WebRTCStreamer::render_frame(
        const cv::Mat& frame
    )
    {
        return impl != nullptr && impl->push_frame(frame);
    };

    bool WebRTCStreamer::is_running() const
    {
        return impl != nullptr && impl->running.load(std::memory_order_acquire);
    };

    bool WebRTCStreamer::has_client() const
    {
        return impl != nullptr && impl->has_client();
    };

    std::string WebRTCStreamer::browser_url() const
    {
        if (impl == nullptr)
        {
            return {};
        }

        return (
            std::string{"http://"} +
            impl->config.host + ":" +
            std::to_string(impl->config.port) +
            "/"
        );
    };
} // namespace visualization
