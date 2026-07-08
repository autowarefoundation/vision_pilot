#include <visualization/webrtc_stream.hpp>

// GStreamer headers for WebRTC streaming
#include <gst/app/gstappsrc.h>
#include <gst/sdp/gstsdpmessage.h>
#include <gst/webrtc/webrtc.h>

// libsoup for WebSocket signaling
#include <libsoup/soup.h>

// JSON-GLib for JSON handling in signaling messages
#include <json-glib/json-glib.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <atomic>
#include <cstdint>


namespace visualization {

    namespace {

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


        // Helper func to initialize GStreamer once (thread-safe)
        void init_gstreamer_once() {

            static std::once_flag once;
            std::call_once(once, []() {
                gst_init(nullptr, nullptr);
            });

        }


        // Helper func to escape JSON strings for signaling messages
        std::string escape_json(
            const std::string& value
        ) {
            gchar * escaped = g_strescape(value.c_str(), nullptr);
            std::string result = escaped != nullptr ? escaped : "";
            g_free(escaped);

            return result;
        };


        // Helper func to generate JSON signaling message for SDP offer
        std::string make_offer_message(
            const std::string& sdp_offer
        ) {

            return  std::string{"{ \"type\": \"offer\", \"sdp\": \""} + \
                    escape_json(sdp_offer) + "\" }";

        };


        // Helper func to generate JSON signaling message for ICE candidate
        std::string make_candidate_message(
            int sdp_mline_index,
            const std::string& candidate
        ) {

            return  std::string{"{ \"type\": \"candidate\", \"sdpMLineIndex\": "} + \
                    std::to_string(sdp_mline_index) + \
                    ", \"candidate\": \"" + \
                    escape_json(candidate) + \
                    "\" }";

        };


        // Helper func to ensure all received frames are in BGR format for WebRTC streaming
        cv::Mat ensure_bgr_frame(
            const cv::Mat& frame
        ) {

            // Return empty frame as is
            if (frame.empty()) {
                return frame;
            };

            // If already in BGR format, return as is (or clone if not continuous)
            if (frame.type() == CV_8UC3) {
                return frame.isContinuous() ? frame : frame.clone();
            };

            // Convert grayscale or BGRA frames to BGR format for WebRTC streaming
            cv::Mat converted;
            if (frame.type() == CV_8UC1) {          // If grayscale
                cv::cvtColor(
                    frame, 
                    converted, 
                    cv::COLOR_GRAY2BGR
                );
            } else if (frame.type() == CV_8UC4) {   // If BGRA
                cv::cvtColor(
                    frame,
                    converted,
                    cv::COLOR_BGRA2BGR
                );
            } else {                                // For other formats, try direct conversion
                frame.convertTo(
                    converted,
                    CV_8UC3
                );
                if (converted.channels() != 3) {
                    return cv::Mat();
                };
            };

            return converted;

        };


    };  // namespace


    // Implementation of WebRTCStreamer class methods
    struct WebRTCStreamer::Impl {

        // Implementation details for WebRTC streaming (e.g., GStreamer pipeline, signaling, etc.)
        // This struct shall contain members and methods to manage WebRTC connections, encode frames, handle signaling, etc.

        explicit Impl(Config config_in) : config(std::move(config_in)) {};


        // Helper funcs for WebRTC streaming handling
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


        // Config for WebRTC streaming
        Config config;
        SoupServer *server = nullptr;
        GMainLoop *main_loop = nullptr;
        std::thread server_thread;


        // GStreamer elements for WebRTC streaming
        GstElement *pipeline = nullptr;
        GstElement *appsrc = nullptr;
        GstElement *webrtc = nullptr;


        // State management for signaling and streaming
        mutable std::mutex signal_mutex;
        SoupWebsocketConnection *client_connection = nullptr;
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

    };


    // HTTP handler for root path to serve browser page with WebRTC client
    void root_http_handler(
        SoupServer *server,
        SoupMessage *msg,
        const char *path,
        GHashTable *query,
        SoupClientContext *client,
        gpointer user_data
    ) {

        (void)server;
        (void)path;
        (void)query;
        (void)user_data;

        // Server a simple HTML page with JS to link WS and display video stream via WebRTC
        soup_message_set_response(
            msg,
            "text/html", 
            SOUP_MEMORY_STATIC,
            kBrowserHtml,
            std::strlen(kBrowserHtml)
        );

    };


    // Websocket handler for closing connection to clean up client state
    void on_websocket_closed(
        SoupWebsocketConnection *connection,
        gpointer user_data
    ) {

        auto *impl = static_cast<WebRTCStreamer::Impl*>(user_data);
        std::lock_guard<std::mutex> lock(impl->signal_mutex);

        if (impl->client_connection == connection) {
            g_object_unref(impl->client_connection);
            impl->client_connection = nullptr;
        }

    };


    // Websocket handler for remote candidate
    void handle_remote_candidate(
        WebRTCStreamer::Impl *impl,
        int sdp_mline_index,
        const std::string& candidate
    ) {

        g_signal_emit_by_name(
            impl->webrtc,
            "add-ice-candidate",
            static_cast<guint>(sdp_mline_index),
            candidate.c_str()
        );

    };


    // Websocket handler for remote description (SDP answer)
    void handle_remote_description(
        WebRTCStreamer::Impl *impl, 
        const std::string & sdp_text
    ) {

        // Parse SDP text into GstSDPMessage
        GstSDPMessage *sdp = nullptr;
        if (
            (gst_sdp_message_new_from_text(sdp_text.c_str(), &sdp) != GST_SDP_OK) || 
            (sdp == nullptr)
        ) {
            return;
        }

        // Set remote description on webrtcbin element
        GstWebRTCSessionDescription *answer = gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
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


    // Websocket handler for incoming messages (SDP offers and ICE candidates)
    void on_websocket_message(
        SoupWebsocketConnection *connection,
        gint type,
        GBytes *message,
        gpointer user_data
    ) {

        (void)connection;
        (void)type;

        auto *impl = static_cast<WebRTCStreamer::Impl*>(user_data);
        gsize size = 0;
        const gchar *data = static_cast<const gchar *>(g_bytes_get_data(message, &size));
        
        // Handle empty messages gracefully
        if (data == nullptr || size == 0) {
            return;
        };

        JsonParser *parser = json_parser_new();
        GError *error = nullptr;
        
        // Handle JSON parsing errors gracefully
        if (!json_parser_load_from_data(parser, data, static_cast<gsize>(size), &error)) {
            if (error != nullptr) {
                g_error_free(error);
            }
            g_object_unref(parser);
            return;
        };

        JsonNode *root = json_parser_get_root(parser);
        JsonObject *object = json_node_get_object(root);
        
        // Handle missing object or type field gracefully
        if (object == nullptr || !json_object_has_member(object, "type")) {
            g_object_unref(parser);
            return;
        };

        
        // Handle signaling messages based on their type (SDP offers and ICE candidates)
        const gchar *signal_type = json_object_get_string_member(object, "type");

        // Handle SDP answer messages to set remote description
        if (
            (g_strcmp0(signal_type, "answer") == 0) && 
            (json_object_has_member(object, "sdp"))
        ) {
            handle_remote_description(
                impl, 
                json_object_get_string_member(object, "sdp")
            );
        // Handle ICE candidate messages
        } else if (
            (g_strcmp0(signal_type, "candidate") == 0) && 
            (json_object_has_member(object, "candidate")) && 
            (json_object_has_member(object, "sdpMLineIndex"))
        ) {
            const int sdp_mline_index = json_object_get_int_member(object, "sdpMLineIndex");
            const std::string candidate = json_object_get_string_member(object, "candidate");

            // If remote description is already set, add candidate immediately
            if (impl->remote_description_ready.load(std::memory_order_acquire)) {
                handle_remote_candidate(impl, sdp_mline_index, candidate);
            // Otherwise, queue candidate to be added once remote description is set
            } else {
                impl->queue_remote_candidate(sdp_mline_index, candidate);
            }
        };

        g_object_unref(parser);

    };


    // Websocket handler for new connections to set up signaling handlers and manage client state
    void websocket_handler(
        SoupServer *server,
        SoupWebsocketConnection *connection,
        const char *path,
        SoupClientContext *client,
        gpointer user_data
    ) {
        
        (void)server;
        (void)path;

        auto *impl = static_cast<WebRTCStreamer::Impl *>(user_data);

        // Set up handlers for incoming messages and connection closure
        g_signal_connect(
            connection, 
            "message", 
            G_CALLBACK(on_websocket_message), 
            impl
        );
        g_signal_connect(
            connection, 
            "closed", 
            G_CALLBACK(on_websocket_closed), 
            impl
        );

        // Set keepalive interval to detect dead connections faster
        soup_websocket_connection_set_keepalive_interval(connection, 15);

        g_object_ref(connection);

        // Update client connection state in a thread-safe manner
        {
            std::lock_guard<std::mutex> lock(impl->signal_mutex);
            if (impl->client_connection != nullptr) {
                g_object_unref(impl->client_connection);
            }
            impl->client_connection = connection;
        }

        impl->flush_pending_signals();

    };


    // Callback for when SDP offer is created to set local description and send offer to client
    void on_offer_created(
        GstPromise *promise, 
        gpointer user_data
    ) {

        auto *impl = static_cast<WebRTCStreamer::Impl *>(user_data);
        const GstStructure *reply = gst_promise_get_reply(promise);
        GstWebRTCSessionDescription *offer = nullptr;

        if (reply != nullptr) {
            gst_structure_get(
                reply, 
                "offer", 
                GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, 
                NULL
            );
        }

        if (offer == nullptr) {
            gst_promise_unref(promise);
            return;
        }

        g_signal_emit_by_name(
            impl->webrtc, 
            "set-local-description", 
            offer, 
            nullptr
        );

        gchar *sdp_text = gst_sdp_message_as_text(offer->sdp);
        if (sdp_text != nullptr) {
            impl->queue_signal(make_offer_message(sdp_text));
            g_free(sdp_text);
        }

        gst_webrtc_session_description_free(offer);
        gst_promise_unref(promise);

    };


    // Callback for when negotiation is needed to create a new SDP offer
    void on_negotiation_needed(
        GstElement *element, 
        gpointer user_data
    ) {

        (void)element;

        auto *impl = static_cast<WebRTCStreamer::Impl *>(user_data);
        GstPromise *promise = gst_promise_new_with_change_func(
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

    
    // Callback for when an ICE candidate is gathered to send it to the client
    void on_ice_candidate(
        GstElement *element, 
        guint mline_index, 
        gchar *candidate, 
        gpointer user_data
    ) {
        
        (void)element;
        auto *impl = static_cast<WebRTCStreamer::Impl *>(user_data);

        if (candidate != nullptr) {
            impl->queue_signal(make_candidate_message(
                static_cast<int>(mline_index), 
                candidate
            ));
        };

    };


    // Full implementation of WebRTCStreamer
    bool WebRTCStreamer::Impl::start()
    {
        
        // =========== 1. Init GStreamer (thread-safe)
        init_gstreamer_once();

        // =========== 2. Init SoupServer for signaling

        // a. Init server instance
        server = soup_server_new(
            "server-header", 
            "VisionPilot", 
            NULL
        );
        if (server == nullptr) {
            return false;
        };
            g_printerr("[WebRTCStreamer] soup_server created\n");

        // b. Listen on specified port and handle errors gracefully
        GError *listen_error = nullptr;
        if (!soup_server_listen_local(
            server, 
            config.port, 
            SOUP_SERVER_LISTEN_IPV4_ONLY, 
            &listen_error
        )) {
            if (listen_error != nullptr) {
                g_printerr("[WebRTCStreamer] failed to listen on port %d: %s\n", config.port, listen_error->message);
                g_error_free(listen_error);
            } else {
                g_printerr("[WebRTCStreamer] failed to listen on port %d (unknown error)\n", config.port);
            }
            return false;
        };

        // c. Add HTTP handler for root path and WebSocket handler for signaling
        soup_server_add_handler(
            server, 
            "/", 
            root_http_handler, 
            this, 
            nullptr
        );
        soup_server_add_websocket_handler(
            server, 
            config.websocket_path.c_str(), 
            nullptr, 
            nullptr, 
            websocket_handler, 
            this, 
            nullptr
        );
        g_printerr("[WebRTCStreamer] soup_server listening on port %d and handlers installed\n", config.port);

        // =========== 3. Start GMainLoop in a separate thread to handle server events
        
        // a. Init and rollout main loop
        main_loop = g_main_loop_new(nullptr, FALSE);
        server_thread = std::thread([this]() {
            g_main_loop_run(main_loop);
        });

        // b. Wait a lil bit for server to start and print sum status
        GError *pipeline_error = nullptr;
        pipeline = gst_parse_launch(
            "appsrc name=source is-live=true format=time do-timestamp=true block=true ! "
            "queue leaky=downstream max-size-buffers=2 ! videoconvert ! vp8enc ! "
            "rtpvp8pay pt=96 ! application/x-rtp,media=video,encoding-name=VP8,payload=96,clock-rate=90000 ! "
            "webrtcbin name=webrtc bundle-policy=max-bundle",
            &pipeline_error
        );

        // c. Handle errors gracefully
        if (pipeline == nullptr) {
            if (pipeline_error != nullptr) {
                g_printerr("[WebRTCStreamer] gst_parse_launch failed: %s\n", pipeline_error->message);
                g_error_free(pipeline_error);
            } else {
                g_printerr("[WebRTCStreamer] gst_parse_launch failed (no error provided)\n");
            }
            stop();
            return false;
        }

        // d. Get references to appsrc and webrtcbin elements and handle errors gracefully
        appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "source");
        webrtc = gst_bin_get_by_name(GST_BIN(pipeline), "webrtc");
        g_printerr("[WebRTCStreamer] pipeline created, appsrc=%p webrtc=%p\n", appsrc, webrtc);

        if (
            (appsrc == nullptr) || 
            (webrtc == nullptr)
        ) {
            stop();
            return false;
        }

        g_object_set(G_OBJECT(appsrc), "block", TRUE, nullptr);
        g_signal_connect(webrtc, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), this);
        g_signal_connect(webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate), this);

        // =========== 4. Set pipeline to PLAYING state and handle errors gracefully

        if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
            g_printerr("[WebRTCStreamer] gst_element_set_state PLAYING failed\n");
            stop();
            return false;
        }

        g_printerr("[WebRTCStreamer] pipeline set to PLAYING\n");

        running.store(true, std::memory_order_release);

        return true;

    };


    // Stop streaming, clean up resources, and shut down server
    bool WebRTCStreamer::Impl::stop()
    {
        
        // Stop streaming loop, prevent new frames from being pushed
        running.store(false, std::memory_order_release);

        if (pipeline != nullptr) {
            gst_element_set_state(pipeline, GST_STATE_NULL);
        }

        if (main_loop != nullptr) {
            g_main_loop_quit(main_loop);
        }

        if (server_thread.joinable()) {
            server_thread.join();
        }

        // Thread-safely clean up GStreamer elements and server resources
        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            if (client_connection != nullptr) {
                g_object_unref(client_connection);
                client_connection = nullptr;
            }
        }

        if (appsrc != nullptr) {
            gst_object_unref(appsrc);
            appsrc = nullptr;
        }

        if (webrtc != nullptr) {
            gst_object_unref(webrtc);
            webrtc = nullptr;
        }

        if (pipeline != nullptr) {
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }

        if (server != nullptr) {
            soup_server_disconnect(server);
            g_object_unref(server);
            server = nullptr;
        }

        if (main_loop != nullptr) {
            g_main_loop_unref(main_loop);
            main_loop = nullptr;
        }

        return true;

    };


    // Pushes frame to GStreamer pipeline for encoding and streaming via WebRTC
    // Also handles format conversion and timestamping
    bool WebRTCStreamer::Impl::push_frame(
        const cv::Mat & frame
    ) {

        // Validate state and input frame before processing
        if (
            (!running.load(std::memory_order_acquire)) || 
            (appsrc == nullptr) || 
            (frame.empty())
        ) {
            return false;
        }

        // Ensure frame is in BGR format for WebRTC streaming
        cv::Mat bgr_frame = ensure_bgr_frame(frame);
        if (bgr_frame.empty()) {
            return false;
        }

        // Configure caps on appsrc if not already configured or if frame size changes
        if (
            (!caps_configured) || 
            (bgr_frame.cols != configured_width) || 
            (bgr_frame.rows != configured_height)
        ) {
            GstCaps *caps = gst_caps_new_simple(
                "video/x-raw",
                "format", G_TYPE_STRING, "BGR",
                "width", G_TYPE_INT, bgr_frame.cols,
                "height", G_TYPE_INT, bgr_frame.rows,
                nullptr
            );

            if (config.frame_rate > 0.0) {
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

        // Create new GstBuffer and copy frame data into it
        const std::size_t payload_size = static_cast<std::size_t>(bgr_frame.total() * bgr_frame.elemSize());
        GstBuffer *buffer = gst_buffer_new_allocate(nullptr, payload_size, nullptr);
        if (buffer == nullptr) {
            return false;
        }

        GstMapInfo map;
        if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
            gst_buffer_unref(buffer);
            return false;
        }

        std::memcpy(
            map.data, 
            bgr_frame.data, 
            payload_size
        );
        gst_buffer_unmap(buffer, &map);

        // Set PTS, DTS, and duration on buffer based on config frame rate
        const guint64 duration_ns = config.frame_rate > 0.0
            ? static_cast<guint64>(GST_SECOND / config.frame_rate)
            : GST_CLOCK_TIME_NONE;
        const guint64 pts_ns = (
            duration_ns == GST_CLOCK_TIME_NONE ? 
            0 : 
            frame_index.fetch_add(1, std::memory_order_acq_rel) * duration_ns
        );
        GST_BUFFER_PTS(buffer) = pts_ns;
        GST_BUFFER_DTS(buffer) = pts_ns;
        GST_BUFFER_DURATION(buffer) = duration_ns;

        return (gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer) == GST_FLOW_OK);

    };


    // Check if there is ctive client connection for streaming
    bool WebRTCStreamer::Impl::has_client() const
    {

        std::lock_guard<std::mutex> lock(signal_mutex);

        return (
            (client_connection != nullptr) && 
            (soup_websocket_connection_get_state(client_connection) == SOUP_WEBSOCKET_STATE_OPEN)
        );

    };


    // Queue signaling message to be sent to client when connection is available
    // Or store in pending queue if not ready yet
    void WebRTCStreamer::Impl::queue_signal(
        const std::string & signal
    ) {
        
        std::lock_guard<std::mutex> lock(signal_mutex);
        
        if (
            (client_connection != nullptr) && 
            (soup_websocket_connection_get_state(client_connection) == SOUP_WEBSOCKET_STATE_OPEN)
        ) {
            soup_websocket_connection_send_text(
                client_connection, 
                signal.c_str()
            );

            return;
        }

        pending_signals.push_back(signal);

    }

    
    // Flush any pending signaling messages that were queued before client connection was ready
    void WebRTCStreamer::Impl::flush_pending_signals()
    {
        
        std::vector<std::string> queued_signals;
        SoupWebsocketConnection *connection = nullptr;

        {
            std::lock_guard<std::mutex> lock(signal_mutex);
            if (
                (client_connection == nullptr) || 
                (soup_websocket_connection_get_state(client_connection) != SOUP_WEBSOCKET_STATE_OPEN) || 
                (pending_signals.empty())
            ) {
                return;
            }

            queued_signals.swap(pending_signals);
            connection = client_connection;

        }

        for (const auto & signal : queued_signals) {
            soup_websocket_connection_send_text(
                connection, 
                signal.c_str()
            );
        }

    };


    // Queue remote ICE candidate to be added once remote description is set
    // Or add immediately if already ready
    void WebRTCStreamer::Impl::queue_remote_candidate(
        int sdp_mline_index, 
        const std::string & candidate
    ) {

        std::lock_guard<std::mutex> lock(remote_candidate_mutex);
        pending_remote_candidates.emplace_back(
            sdp_mline_index, 
            candidate
        );

    };

    
    // Flush any pending remote ICE candidates that were received before remote description was set
    void WebRTCStreamer::Impl::flush_pending_remote_candidates()
    {
        
        std::vector<std::pair<int, std::string>> queued_candidates;
        
        {
            std::lock_guard<std::mutex> lock(remote_candidate_mutex);
            queued_candidates.swap(pending_remote_candidates);
        }

        for (const auto & candidate : queued_candidates) {
            handle_remote_candidate(
                this, 
                candidate.first, 
                candidate.second
            );
        }

    };


    // WebRTCStreamer constructor and destructor and aux. funcs

    WebRTCStreamer::WebRTCStreamer(Config config) : impl(std::make_unique<Impl>(std::move(config))) {}

    WebRTCStreamer::WebRTCStreamer() : WebRTCStreamer(Config()) {}

    bool WebRTCStreamer::init(Config config)
    {
        if (impl != nullptr) {
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
        
        stop();

    }

    bool WebRTCStreamer::start()
    {
        
        return impl != nullptr && impl->start();

    }

    bool WebRTCStreamer::stop()
    {
        
        if (impl != nullptr) {
            return impl->stop();
        }

        return true;

    };

    bool WebRTCStreamer::render_frame(
        const cv::Mat & frame
    ) {
        
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
        
        if (impl == nullptr) {
            return {};
        }

        return (
            std::string{"http://"} + \
            impl->config.host + ":" + \
            std::to_string(impl->config.port) + \
            "/"
        );

    };


}   // namespace visualization