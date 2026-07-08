#ifndef VISIONPILOT_VISUALIZATION_TO_WEBRTC_H
#define VISIONPILOT_VISUALIZATION_TO_WEBRTC_H

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <visualization/visual_interface.hpp>


namespace visualization {

    class WebRTCStreamer : public VisualInterface {

        public:
            

            /**
            * @brief Config options for the WebRTC streamer.
            * Provides parameters for WebRTC connection and streaming behavior.
            * 
            * Includes:
            * - host: WebRTC signaling server host (default: "127.0.0.1")
            * - port: WebRTC signaling server port (default: 8080)
            * - websocket_path: WebRTC signaling server WebSocket path (default: "/ws")
            * - frame_rate: desired streaming frame rate in FPS (default: 10.0 FPS)
            */
            struct Config {
                std::string host = "127.0.0.1"; // Default to IPv4 localhost
                uint16_t port = 8080;
                std::string websocket_path = "/ws";
                double frame_rate = 10.0;       // Default to 10 FPS
            };


            /**
            * @brief Constructor for WebRTCStreamer.
            * Inits WebRTC streamer with specified config.
            *
            * @param config Config options for WebRTC connection and streaming behavior.
            */
            WebRTCStreamer();
            explicit WebRTCStreamer(Config config);


            /**
            * @brief Initialize and start the WebRTC streaming session.
            * Creates the signaling server, GStreamer pipeline, and browser-facing endpoint.
            *
            * @param config Config options for WebRTC connection and streaming behavior.
            * @return true if initialization succeeded, false otherwise.
            */
            bool init(Config config);


            /**
            * @brief Convenience overload that initializes WebRTC with default config,
            * except for the supplied browser port.
            *
            * @param port Browser/signaling server port.
            * @return true if initialization succeeded, false otherwise.
            */
            bool init(uint16_t port);


            /**
            * @brief Destructor for WebRTCStreamer.
            * Cleans up WebRTC resources and connections.
            */
            ~WebRTCStreamer();
            WebRTCStreamer(const WebRTCStreamer&) = delete;
            WebRTCStreamer& operator=(const WebRTCStreamer&) = delete;


            // STREAM HANDLING FUNCS

            
            /**
            * @brief Start the WebRTC streaming session.
            * Establishes connection to signaling server and prepares for streaming.
            *
            * @return true if streaming started successfully, false otherwise
            */
            bool start();


            /**
            * @brief Stop the WebRTC streaming session.
            * Closes WebRTC connections and cleans up resources.
            *
            * @return true if streaming stopped successfully, false otherwise
            */
            bool stop();

            
            /**
            * @brief Push a new video frame to the WebRTC stream.
            * Provides a thread-safe way to send frames to connected WebRTC clients.
            * 
            * @param frame The video frame to stream (as cv::Mat)
            *
            * @return true if frame was successfully pushed to the stream, false otherwise
            */
            bool render_frame(const cv::Mat& frame) override;


            /**
            * @brief Check if WebRTC stream is currently running.
            *
            * @return true if streaming is active, false otherwise
            */
            bool is_running() const;


            /**
            * @brief Check if there are any connected WebRTC clients.
            *
            * @return true if at least one client is connected, false otherwise
            */
            bool has_client() const;


            /**
            * @brief Get URL for browser to connect to WebRTC stream.
            *
            * @return Browser URL for WebRTC stream
            */
            std::string browser_url() const;


            /**
            * @brief Internal implementation details for WebRTC streamer.
            * Kinda manages WebRTC connection management, frame encoding, and streaming logic etc.
            */
            struct Impl;


        private:


            // Internal implementation details (WebRTC connection, encoding, etc.)
            std::unique_ptr<Impl> impl;
    

        };

};  // namespace visualization


#endif //VISIONPILOT_VISUALIZATION_TO_WEBRTC_H
