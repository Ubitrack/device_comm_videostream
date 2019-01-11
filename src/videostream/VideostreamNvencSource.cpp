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
 * VideostreamNvencSource component.
 *
 * @author Ulrich Eck
 */

// std
#include <string>

// on windows, asio must be included before anything that possible includes windows.h
// don't ask why.
#include <boost/asio.hpp>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>

#include <utVision/Image.h>
#include <opencv/highgui.h>

#include <NvPipe.h>

#include "VideostreamNvencProtocol.h"

#include <log4cpp/Category.hh>

static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.VideostreamNvencSource" ) );

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
        class VideostreamNvencSourceComponent
                : public Dataflow::Component
        {
            // consumer port
            Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

            int m_listenPort;

            // boost asio service
            boost::shared_ptr< boost::asio::io_service > m_ioService;

            // the socket for receiving image information
            boost::shared_ptr< boost::asio::ip::udp::socket > m_socket;
            boost::asio::ip::udp::endpoint m_endpoint;

            // Recive data. Do not touch from outside of async network thread
            enum { max_receive_length = 1024000, receive_buffer_size = 1024200 };
            char receive_data[receive_buffer_size];


            boost::shared_ptr< boost::thread > m_pNetworkThread;

            boost::shared_ptr<NvPipe> m_video_decoder;

            NvPipe_Codec m_video_codec;

            NvPipe_Compression m_video_compression;

            NvPipe_Format m_video_format;

            std::vector<uint8_t> m_receive_buffer;

        public:

            /** constructor */
            VideostreamNvencSourceComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
                    : Dataflow::Component( name )
                    , m_outPort( "Output", *this )
                    , m_listenPort( 0x5554 ) // default port is 0x5554 (UT) 21844
            {

                // check for configuration
                if( pConfig->m_DataflowAttributes.hasAttribute( "networkPort" ) )
                {
                     pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_listenPort );
                }



            }

            virtual void start()
            {
                using boost::asio::ip::udp;

                LOG4CPP_DEBUG( logger, "Starting Network Source service" );
                LOG4CPP_DEBUG( logger, "Creating receiver on port " << m_listenPort );

                m_ioService.reset( new boost::asio::io_service );

                m_socket.reset( new udp::socket( *m_ioService ) );

                m_socket->open( udp::v4() );
                boost::asio::socket_base::reuse_address option( true );
                m_socket->set_option( option );
                m_socket->bind( udp::endpoint( udp::v4(), m_listenPort ) );

                m_socket->async_receive_from(
                        boost::asio::buffer( receive_data, max_receive_length ),
                        m_endpoint,
                        boost::bind( &VideostreamNvencSourceComponent::HandleReceive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );

                // network thread runs until io_service is interrupted
                LOG4CPP_DEBUG( logger, "Starting network receiver thread" );
                m_pNetworkThread = boost::shared_ptr< boost::thread >( new boost::thread( boost::bind( &boost::asio::io_service::run, m_ioService.get() ) ) );

                LOG4CPP_DEBUG( logger, "Network Source service started" );

            }

            virtual void stop()
            {

                if ( m_running )
                {
                    m_running = false;
                    LOG4CPP_NOTICE( logger, "Stopping Network Source Module" );

                    if ( m_ioService )
                    {
                        LOG4CPP_TRACE( logger, "Stopping IO service" );
                        m_ioService->stop();

                        LOG4CPP_TRACE( logger, "Joining network thread" );
                        if ( m_pNetworkThread )
                            m_pNetworkThread->join();

                        LOG4CPP_TRACE( logger, "Detroying network thread" );
                        m_pNetworkThread.reset();

                        LOG4CPP_TRACE( logger, "Closing socket" );
                        if ( m_socket )
                        {
                            m_socket->close();
                            m_socket.reset();
                        }

                        LOG4CPP_TRACE( logger, "Detroying IO service" );
                        m_ioService.reset();
                    }

                }
                LOG4CPP_DEBUG( logger, "Network Source Stopped" );
            }

        protected:


            void HandleReceive( const boost::system::error_code err, size_t length ) {
                Measurement::Timestamp recvtime = Measurement::now();

                LOG4CPP_DEBUG(logger, "Received " << length << " bytes");

                // some error checking
                if (err && err != boost::asio::error::message_size) {
                    std::ostringstream msg;
                    msg << "Error receiving from socket: \"" << err << "\"";
                    LOG4CPP_ERROR(logger, msg.str());
                    UBITRACK_THROW(msg.str());
                }

                if (length >= max_receive_length) {
                    LOG4CPP_ERROR(logger, "Too many bytes received");
                    UBITRACK_THROW("FIXME: received more than max_receive_length bytes.");
                }

                try {

                    VideostreamNvencProtocol header;

                    std::vector<uint8_t> hdr_recbuf(receive_data, receive_data + header.size());
                    header.copy_from(hdr_recbuf);

                    Measurement::Timestamp sendtime = header.timestamp();

                    if (header.framesize() > 0) {
                        if (length != header.framesize() + header.size()) {
                            LOG4CPP_ERROR(logger, "Unexpected length of package: " << length << " should be "
                                                                                   << header.framesize() +
                                                                                      header.size());
                            return;
                        }

                        // Currently only works with BGRA32
                        if (header.format() != NVPIPE_BGRA32) {
                            LOG4CPP_ERROR(logger, "Decoder only supports BGRA32 for now.");
                            return;
                        }

                        // create decoder instance if needed
                        if (!m_video_decoder) {
                            m_video_decoder.reset(NvPipe_CreateDecoder(m_video_format, m_video_codec));
                            if (!m_video_decoder) {
                                LOG4CPP_ERROR(logger, "Failed to create decoder: " << NvPipe_GetError(NULL));
                                return;
                            }
                        }

                        Vision::Image::ImageFormatProperties props;

                        size_t elementSize = 32;
                        switch (header.format()) {
                            case NVPIPE_BGRA32:
                                props.imageFormat = Vision::Image::BGRA;
                                props.bitsPerPixel = 32;
                                props.channels = 4;
                                props.matType = CV_8UC4;
                                props.depth = CV_8U;
                                elementSize = 4;
                                break;
//                        case NVPIPE_UINT4:
//                            elementSize = 4;
//                            break;
                            case NVPIPE_UINT8:
                                props.imageFormat = Vision::Image::LUMINANCE;
                                props.bitsPerPixel = 8;
                                props.channels = 1;
                                props.matType = CV_8UC1;
                                props.depth = CV_8U;
                                elementSize = 1;
                                break;
                            case NVPIPE_UINT16:
                                props.imageFormat = Vision::Image::LUMINANCE;
                                props.bitsPerPixel = 16;
                                props.channels = 1;
                                props.matType = CV_16UC1;
                                props.depth = CV_16U;
                                elementSize = 2;
                                break;
                            case NVPIPE_UINT32:
                                props.imageFormat = Vision::Image::LUMINANCE;
                                props.bitsPerPixel = 32;
                                props.channels = 1;
                                props.matType = CV_32SC1;
                                props.depth = CV_32S;
                                elementSize = 4;
                                break;
                            default: LOG4CPP_ERROR(logger, "Unknown format: " << header.format());

                        }

                        Image::Ptr currentImage(new Image(header.width(), header.height(), props));


                        std::vector<uint8_t> img_recbuf(&receive_data[header.size()], header.framesize());

                        uint64_t r = NvPipe_Decode(m_video_decoder.get(), img_recbuf.data(), header.framesize(),
                                                   currentImage->Mat().data, header.width(), header.height());
                        if (0 == r) {
                            LOG4CPP_ERROR(logger, "Decode error: " << NvPipe_GetError(m_video_decoder.get()));
                            return;
                        }


                        m_outPort.send(Measurement::ImageMeasurement(sendtime, currentImage));
                    }
                }
                catch (const std::exception &e) {
                    LOG4CPP_ERROR(logger, "Caught exception " << e.what());
                }

                // restart receiving new packet
                m_socket->async_receive_from(
                        boost::asio::buffer(receive_data, max_receive_length),
                        m_endpoint,
                        boost::bind(&VideostreamNvencSourceComponent::HandleReceive, this, boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred)
                );

            }

        };

// register module at factory
        UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
            cf->registerComponent< VideostreamNvencSourceComponent > ( "VideostreamNvencSource" );

        }

    } } // namespace Ubitrack::Vision
