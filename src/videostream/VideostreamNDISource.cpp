/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 * contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup driver_components
 * @file
 * VideostreamNDISource component.
 *
 * @author Ulrich Eck
 */

//#ifdef HAVE_NDI

// std
#include <string>
#include <chrono>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

#include <log4cpp/Category.hh>

#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>

#include <utVision/Image.h>

#include <Processing.NDI.Lib.h>


#include "VideostreamNDIContext.h"


static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.VideostreamNDISource" ) );

namespace Ubitrack { namespace Vision {



/**
 * @ingroup dataflow_components
 * Transmits measurements over the network.
 *
 * @par Input Ports
 * PushConsumer<EventType> port "Input"
 *
 * @par Output Ports
 * None.
 *
 * @par Configuration
 * - Edge configuration:
 * @verbatim
 * <Configuration port="..." destination="..."/>
 * @endverbatim
 *   - \c port: the TCP target port, default 0x5554 ("UT")
 *   - \c destination: the target machine or ip, default 127.0.0.1
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::ImageMeasurement
 */
        class VideostreamNDISourceComponent
                : public Dataflow::Component
        {
            // consumer port
            Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;


            std::string m_sink_short_name;
            std::string m_sink_session;

            boost::shared_ptr< boost::thread > m_pNetworkThread;


        public:

            /** constructor */
            VideostreamNDISourceComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
                    : Dataflow::Component( name )
                    , m_outPort( "Output", *this )
                    , m_sink_short_name("videosource")
                    , m_sink_session("default")
            {

                // check for configuration
                if ( pConfig->m_DataflowAttributes.hasAttribute( "sinkShortName" ) )
                {
                    m_sink_short_name = pConfig->m_DataflowAttributes.getAttributeString( "sinkShortName" );
                }

                if ( pConfig->m_DataflowAttributes.hasAttribute( "sinkSession" ) )
                {
                    m_sink_session = pConfig->m_DataflowAttributes.getAttributeString( "sinkSession" );
                }

            }

            virtual void start()
            {
                if (!m_running) {
                    // network thread runs m_isrunning
                    LOG4CPP_DEBUG( logger, "Starting network receiver thread" );
                    m_running = true;
                    m_pNetworkThread = boost::shared_ptr< boost::thread >( new boost::thread( boost::bind( &VideostreamNDISourceComponent::MainLoop, this ) ) );

                    LOG4CPP_DEBUG( logger, "Network Source service started" );
                }
            }

            virtual void stop()
            {

                if ( m_running )
                {
                    m_running = false;
                    LOG4CPP_NOTICE( logger, "Stopping Network Source Module" );

                    m_pNetworkThread->join();
                }
                LOG4CPP_DEBUG( logger, "Network Source Stopped" );
            }



            void initialize_ndi()
            {
                static boost::mutex singletonMutex;
                boost::mutex::scoped_lock l(singletonMutex);

                if (!g_ndilib_initialized) {
                    if (!NDIlib_initialize())
                    {	// Cannot run NDI. Most likely because the CPU is not sufficient (see SDK documentation).
                        // you can check this directly with a call to NDIlib_is_supported_CPU()
                        LOG4CPP_ERROR(logger, "Failed to initialize NDI.");
                        return;
                    }
                    g_ndilib_initialized = true;
                }
            }

        protected:

            void MainLoop() {

                initialize_ndi();

                // Create a finder
                NDIlib_find_instance_t pNDI_find = NDIlib_find_create_v2();
                if (!pNDI_find) {
                    LOG4CPP_ERROR(logger, "Could not create NDIlib Find Instance.");
                    m_running = false;
                    return;
                }

                // Wait until there is one source
                uint32_t no_sources = 0;
                const NDIlib_source_t* p_sources = NULL;
                while ((m_running) && (!no_sources))
                {	// Wait until the sources on the nwtork have changed
                    LOG4CPP_INFO(logger, "Looking for NDI sources for 500ms ...");
                    NDIlib_find_wait_for_sources(pNDI_find, 500/* Half a second */);
                    p_sources = NDIlib_find_get_current_sources(pNDI_find, &no_sources);
                }

                // We now have at least one source, so we create a receiver to look at it.
                NDIlib_recv_instance_t pNDI_recv = NDIlib_recv_create_v3();
                if (!pNDI_recv) {
                    LOG4CPP_ERROR(logger, "Could not create NDIlib Receiver Instance.");
                    m_running = false;
                    return;
                }

                bool found_source = false;
                uint32_t selected_source_idx = 0;
                for (uint32_t i = 0; i < no_sources; i++) {
                    LOG4CPP_INFO(logger, "Found NDI Source: " << p_sources[i].p_ndi_name << " at IP: " << p_sources[i].p_ip_address);
                    if (m_sink_short_name.compare(p_sources[i].p_ndi_name) == 0) {
                        selected_source_idx = i;
                        found_source = true;
                        break;
                    }
                }

                if (!found_source) {
                    LOG4CPP_ERROR(logger, "Could not find source: " << m_sink_short_name << " amongst available streams.");
                    m_running = false;
                    return;
                }

                // Connect to our sources
                NDIlib_recv_connect(pNDI_recv, p_sources + 0);

                // Destroy the NDI finder. We needed to have access to the pointers to p_sources[0]
                NDIlib_find_destroy(pNDI_find);

                // Run for one minute
                using namespace std::chrono;

                while (m_running) {
                    NDIlib_video_frame_v2_t video_frame;
                    NDIlib_audio_frame_v2_t audio_frame;

                    switch (NDIlib_recv_capture_v2(pNDI_recv, &video_frame, &audio_frame, nullptr, 500))
                    {	// No data
                        case NDIlib_frame_type_none:
                            LOG4CPP_DEBUG(logger, "No NDI data received for 500ms.");
                            break;

                            // Video data
                        case NDIlib_frame_type_video:
                            LOG4CPP_DEBUG(logger, "Video data received (" << video_frame.xres << "x" << video_frame.yres << ").");
                            ReceiveVideoFrame(video_frame);
                            NDIlib_recv_free_video_v2(pNDI_recv, &video_frame);
                            break;

                            // Audio data
                        case NDIlib_frame_type_audio:
                            LOG4CPP_DEBUG(logger, "Audio data received (%d" << audio_frame.no_samples << " samples)");
                            NDIlib_recv_free_audio_v2(pNDI_recv, &audio_frame);
                            break;
                    }
                }

                // Cleanup
                NDIlib_recv_destroy(pNDI_recv);

                LOG4CPP_INFO(logger, "Exiting NDI Receiver Thread.");
            }

            void ReceiveVideoFrame(const NDIlib_video_frame_v2_t& frame) {

                Ubitrack::Measurement::Timestamp sendtime(frame.timestamp);

                Vision::Image::ImageFormatProperties props;

                size_t elementSize = 32;
                switch (frame.FourCC) {
                    case NDIlib_FourCC_type_BGRA:
                        props.imageFormat = Vision::Image::BGRA;
                        props.bitsPerPixel = 32;
                        props.channels = 4;
                        props.matType = CV_8UC4;
                        props.depth = CV_8U;
                        elementSize = 4;
                        break;
                    case NDIlib_FourCC_type_RGBA:
                        props.imageFormat = Vision::Image::RGBA;
                        props.bitsPerPixel = 32;
                        props.channels = 4;
                        props.matType = CV_8UC4;
                        props.depth = CV_8U;
                        elementSize = 4;
                        break;
                    default:
                        LOG4CPP_ERROR(logger, "Unknown format: " << frame.FourCC);
                        return;
                }

                // create an empty image
                Image::Ptr currentImage(new Image(frame.xres, frame.yres, props));
                cv::Mat img = currentImage->Mat();

                // Copy the frame. It is likely that you would do something much smarter than this.
                for (int y = 0; y < frame.yres; y++)
                {	// The frame data
                    uint8_t* p_image = (uint8_t*)frame.p_data + frame.line_stride_in_bytes*y;

                    int line_idx = y;

                    // Cycle over the line
                    for (int x = 0; x < frame.xres; x++, p_image += 4, line_idx++)
                    {
                        cv::Vec4b pixel = img.at<cv::Vec4b>(y, x);
                        pixel.val[0] = p_image[0];
                        pixel.val[1] = p_image[1];
                        pixel.val[2] = p_image[2];
                        pixel.val[3] = p_image[3];
                    }
                }

                m_outPort.send(Measurement::ImageMeasurement(sendtime, currentImage));
            }

        };

// register module at factory
        UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
            cf->registerComponent< VideostreamNDISourceComponent > ( "VideostreamNDISource" );

        }

    } } // namespace Ubitrack::Vision
//#endif // HAVE_NDI
