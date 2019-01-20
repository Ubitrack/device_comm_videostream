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
 * VideostreamNDISink component.
 *
 * @author Ulrich Eck
 */

//#ifdef HAVE_NDI

// std
#include <csignal>
#include <cstddef>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include <string>
#include <algorithm>
#include <atomic>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

// Ubitrack
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/ComponentFactory.h>

#include <utVision/Image.h>

#include <Processing.NDI.Lib.h>

#include "VideostreamNDIContext.h"

namespace Ubitrack { namespace Vision {

        static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.VideostreamNDISink" ) );

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
 *   - \c long name: Long name of source
 *   - \c short name: Short name of source
 *   - \c session: name of session (default=default)
 *
 * @par Instances
 * Registered for the following EventTypes and names:
 * - Ubitrack::Measurement::ImageMeasurement: NetworkVideostreamNDISink
 */


        class VideostreamNDISinkComponent
                : public Dataflow::Component
        {

            // consumer port
            Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;

            std::string m_sink_long_name;
            std::string m_sink_short_name;
            std::string m_sink_session;

            NDIlib_send_instance_t m_ndi_send;

            boost::shared_ptr<NDIlib_video_frame_v2_t> m_ndi_videoframe;

        public:

            /** constructor */
            VideostreamNDISinkComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
                    : Dataflow::Component( name )
                    , m_inPort( "Input", *this, boost::bind( &VideostreamNDISinkComponent::eventIn, this, _1 ) )
                    , m_ndi_send(nullptr)
                    , m_sink_long_name("Untitled Ubitrack Videosource")
                    , m_sink_short_name("videosource")
                    , m_sink_session("default")
            {
                // check for configurations and reset values if necessary
                if ( pConfig->m_DataflowAttributes.hasAttribute( "sinkLongName" ) )
                {
                    m_sink_long_name = pConfig->m_DataflowAttributes.getAttributeString( "sinkLongName" );
                }

                if ( pConfig->m_DataflowAttributes.hasAttribute( "sinkShortName" ) )
                {
                    m_sink_short_name = pConfig->m_DataflowAttributes.getAttributeString( "sinkShortName" );
                }

                if ( pConfig->m_DataflowAttributes.hasAttribute( "sinkSession" ) )
                {
                    m_sink_session = pConfig->m_DataflowAttributes.getAttributeString( "sinkSession" );
                }

                initialize_ndi();

                // Create an NDI source that is called "My Video" and is clocked to the video.
                NDIlib_send_create_t NDI_send_create_desc;
                NDI_send_create_desc.p_ndi_name = m_sink_short_name.c_str();

                m_ndi_send = NDIlib_send_create(&NDI_send_create_desc);

                // set up connection metadata
                NDIlib_metadata_frame_t NDI_connection_type;

                std::ostringstream cfg_str;
                cfg_str << "<ndi_product long_name=\"" << m_sink_long_name << "\" "
                           "             short_name=\"" << m_sink_short_name << "\" "
                           "             manufacturer=\"Ubitrack.\" "
                           "             version=\"1.000.000\" "
                           "             session=\"\" << m_sink_session << \"\" "
                           "             model_name=\"VideoSink\" "
                           "             serial=\"00000\"/>";
                // why do we need a char* here ??
                NDI_connection_type.p_data = (char*)cfg_str.str().c_str();

                NDIlib_send_add_connection_metadata(m_ndi_send, &NDI_connection_type);

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

            ~VideostreamNDISinkComponent() override {
                // Free the video frame
                if(m_ndi_videoframe) {
                    free(m_ndi_videoframe->p_data);
                }

                // Destroy the NDI sender
                NDIlib_send_destroy(m_ndi_send);



            }

        protected:

            void eventIn( const Measurement::ImageMeasurement& m )
            {

                LOG4CPP_TRACE( logger, "Sending NDI image data timestamp: " << m.time() );

                cv::Mat input_img = m->Mat();
                Vision::Image::ImageFormatProperties img_props;
                m->getFormatProperties(img_props);

                cv::Mat img;
                switch(img_props.imageFormat) {
                    case Vision::Image::BGR:
                        cv::cvtColor(input_img, img, cv::COLOR_BGR2BGRA);
                        break;
                    case Vision::Image::RGB:
                        cv::cvtColor(input_img, img, cv::COLOR_RGB2BGRA);
                        break;
                    case Vision::Image::RGBA:
                        cv::cvtColor(input_img, img, cv::COLOR_RGBA2BGRA);
                        break;
                    case Vision::Image::LUMINANCE:
                        // this could be sent as UINT8 with NVenc
                        cv::cvtColor(input_img, img, cv::COLOR_GRAY2BGRA);
                        break;
                    default:
                        img = input_img;
                }

                if (m_ndi_videoframe == nullptr) {

                    LOG4CPP_INFO(logger, "Creating NDI Videoframe");
                    m_ndi_videoframe = boost::make_shared<NDIlib_video_frame_v2_t>();
                    m_ndi_videoframe->xres = m->width();
                    m_ndi_videoframe->yres = m->height();
                    m_ndi_videoframe->FourCC = NDIlib_FourCC_type_BGRA;
                    m_ndi_videoframe->p_data = (uint8_t*)malloc(m_ndi_videoframe->xres * m_ndi_videoframe->yres * 4);
                    m_ndi_videoframe->line_stride_in_bytes = m->width() * 4;
                }



                if (!NDIlib_send_get_no_connections(m_ndi_send, 10000))
                {	// Display status
                    LOG4CPP_DEBUG(logger, "No current connections, so no rendering needed.");
                }
                else
                {
                    // try locking the ndi-lib interaction
//                    static boost::mutex encodeMutex;
//                    boost::mutex::scoped_lock l(encodeMutex);


                    // Have we received any meta-data
                    NDIlib_metadata_frame_t metadata_desc;
                    if (NDIlib_send_capture(m_ndi_send, &metadata_desc, 0))
                    {	// For example, this might be a connection meta-data string that might include information
                        // about preferred video formats. A full XML parser should be used here, this code is for
                        // illustration purposes only
//                        if (strncasecmp(metadata_desc.p_data, "<ndi_format", 11))
//                        {	// Setup the preferred video format.
//                        }

                        // Display that we got meta-data
                        LOG4CPP_INFO(logger, "Received meta-data : " << metadata_desc.p_data);

                        // We must free the data here
                        NDIlib_send_free_metadata(m_ndi_send, &metadata_desc);
                    }

                    // Get the tally state of this source (we poll it),
                    NDIlib_tally_t NDI_tally;
                    NDIlib_send_get_tally(m_ndi_send, &NDI_tally, 0);

                    // set current timestamp
                    m_ndi_videoframe->timestamp = (long long)m.time();


                    // Fill in the buffer. It is likely that you would do something much smarter than this.
                    for (int y = 0; y < m_ndi_videoframe->yres; y++)
                    {	// The frame data
                        uint8_t* p_image = (uint8_t*)m_ndi_videoframe->p_data + m_ndi_videoframe->line_stride_in_bytes*y;

                        int line_idx = y;

                        // Cycle over the line
                        for (int x = 0; x < m_ndi_videoframe->xres; x++, p_image += 4, line_idx++)
                        {
                            cv::Vec4b pixel = img.at<cv::Vec4b>(y, x);
                            p_image[0] = pixel.val[0];
                            p_image[1] = pixel.val[1];
                            p_image[2] = pixel.val[2];
                            p_image[3] = pixel.val[3];
                        }
                    }

                    // We now submit the frame. Note that this call will be clocked so that we end up submitting at exactly 59.94fps
                    NDIlib_send_send_video_v2(m_ndi_send, &(*m_ndi_videoframe));

                }
            }
        };


// register module at factory
        UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
            cf->registerComponent< VideostreamNDISinkComponent > ( "VideostreamNDISink" );

        }

    } } // namespace Ubitrack::Drivers
//#endif // HAVE_NDI
