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

#include "VideostreamNvencFrameHeader.h"

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
            enum { max_receive_length = UDP_PACKET_SIZE, receive_buffer_size = UDP_PACKET_SIZE+32 }; // why is the buffer larger as the max udp packet ?
            char receive_data[receive_buffer_size];

            // local receive buffer
            std::vector<uint8_t> m_receive_buffer;

            boost::shared_ptr< boost::thread > m_pNetworkThread;

            boost::shared_ptr<NvPipe> m_video_decoder;

            NvPipe_Codec m_video_codec;

            NvPipe_Compression m_video_compression;

            NvPipe_Format m_video_format;

            VideostreamNvencFrameHeader m_last_frame_header;

            size_t m_bytes_received;
            unsigned short m_next_sequence_id;

        public:

            /** constructor */
            VideostreamNvencSourceComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
                    : Dataflow::Component( name )
                    , m_outPort( "Output", *this )
                    , m_listenPort( 0x5554 ) // default port is 0x5554 (UT) 21844
                    , m_last_frame_header()
                    , m_bytes_received(0)
                    , m_next_sequence_id(0)
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

            void resetReceiveState() {
                m_next_sequence_id = 0;
                m_bytes_received = 0;
                m_last_frame_header.mark_invalid();
            }

            void handleReceivedCompleteFrame() {

                if ((m_last_frame_header.is_valid()) && (m_last_frame_header.framesize() > 0)) {
                    Measurement::Timestamp sendtime = m_last_frame_header.timestamp();

                    // Currently only works with BGRA32
                    if (m_last_frame_header.format() != NVPIPE_BGRA32) {
                        LOG4CPP_ERROR(logger, "Decoder only supports BGRA32 for now.");
                        resetReceiveState();
                        return;
                    }
                    m_video_codec = (NvPipe_Codec)m_last_frame_header.codec();
                    m_video_format = (NvPipe_Format)m_last_frame_header.format();

                    // create decoder instance if needed
                    if (!m_video_decoder) {
                        m_video_decoder.reset(NvPipe_CreateDecoder(m_video_format, m_video_codec));
                        if (!m_video_decoder) {
                            LOG4CPP_ERROR(logger, "Failed to create decoder: " << NvPipe_GetError(NULL));
                            resetReceiveState();
                            return;
                        }
                    }

                    Vision::Image::ImageFormatProperties props;

                    size_t elementSize = 32;
                    switch (m_last_frame_header.format()) {
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
                        default:
                            LOG4CPP_ERROR(logger, "Unknown format: " << m_last_frame_header.format());
                    }

                    if (!m_video_decoder) {
                        // should log here
                        LOG4CPP_ERROR(logger, "video-encoder missing.");
                        resetReceiveState();
                        return;
                    }

                    Image::Ptr currentImage(new Image(m_last_frame_header.width(), m_last_frame_header.height(), props));

                    uint64_t r = NvPipe_Decode(m_video_decoder.get(), m_receive_buffer.data(), m_last_frame_header.framesize(),
                                               currentImage->Mat().data, m_last_frame_header.width(), m_last_frame_header.height());
                    if (0 == r) {
                        LOG4CPP_ERROR(logger, "Decode error: " << NvPipe_GetError(m_video_decoder.get()));
                        resetReceiveState();
                        return;
                    }


                    m_outPort.send(Measurement::ImageMeasurement(sendtime, currentImage));

                } else {
                    // received invalid frame
                    LOG4CPP_ERROR(logger, "received invalid frame.");
                    resetReceiveState();
                }
            }

            size_t HandleReceiveHeader(size_t length) {
                size_t header_size = 0;
                if (m_next_sequence_id == 0) {
                    VideostreamNvencFrameHeader header;

                    uint8_t *receive_data_ptr = (uint8_t *) (&receive_data[0]);
                    std::vector<uint8_t> hdr_recbuf(receive_data_ptr, receive_data_ptr + header.size());
                    header.copy_from(hdr_recbuf);
                    if (!header.is_valid()) {
                        LOG4CPP_DEBUG(logger, "Error while receiving packet: header is invalid (frame).");
                        return 0;
                    }

                    LOG4CPP_TRACE(logger, "Received new frame header - seq_id: " << header.seq_id() << " width: " << header.width() <<
                                          " height: " << header.height() << " framesize: " << header.framesize() << " timestamp: " << header.timestamp());

                    // we have received a valid new frame
                    header_size = header.size();
                    m_last_frame_header = header;
                    m_next_sequence_id = (unsigned short)(header.seq_id() + 1);

                    // if send buffer size differs, re-initialize it to message_size_max (with 0)
                    if (m_receive_buffer.size() != header.framesize()) {
                        m_receive_buffer.clear();
                        m_receive_buffer.resize(header.framesize(), 0);
                    }


                } else if (m_last_frame_header.is_valid()) {
                    VideostreamNvencPacketHeader header;

                    auto *receive_data_ptr = (uint8_t *) (&receive_data[0]);
                    std::vector<uint8_t> hdr_recbuf(receive_data_ptr, receive_data_ptr + header.size());
                    header.copy_from(hdr_recbuf);
                    if (!header.is_valid()) {
                        LOG4CPP_DEBUG(logger, "Error while receiving packet: header is invalid (packet).");
                        return 0;
                    }

                    header_size = header.size();
                    m_next_sequence_id = (unsigned short)(header.seq_id() + 1);

                } else {
                    // currently unhandled state ..
                    LOG4CPP_ERROR(logger, "error receiving frame: sequence id != 0 and header invalid.");
                    return 0;
                }
                return header_size;
            }

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

                if (length > max_receive_length) {
                    LOG4CPP_ERROR(logger, "Too many bytes received (max: " << max_receive_length << " actual: " << length);
                    UBITRACK_THROW("FIXME: received more than max_receive_length bytes.");
                }

                size_t header_size = HandleReceiveHeader(length);
                // header_size 0 ==> error
                if (header_size == 0) {
                    // reset state machine
                    LOG4CPP_DEBUG(logger, "received invalid frame (header_size == 0).");
                    resetReceiveState();
                } else {
                    // copy received bytes
                    size_t img_bytes_available = length - header_size;
                    LOG4CPP_TRACE( logger, "Received packet id: \"" << (m_next_sequence_id -1) << " - size: " << (img_bytes_available) << "\"." );

                    if (img_bytes_available + m_bytes_received <= m_receive_buffer.size()) {

                        std::copy(
                                &receive_data[0] + header_size,
                                &receive_data[0] + (header_size + img_bytes_available),
                                m_receive_buffer.data() + m_bytes_received);


                        m_bytes_received += img_bytes_available;

                    } else {
                        // received too much data ..
                        LOG4CPP_ERROR(logger, "received too much data - available: " << img_bytes_available << " received: " << m_bytes_received << " buffer size: " << m_receive_buffer.size());
                        resetReceiveState();
                    }

                    if (m_last_frame_header.is_valid() && (m_bytes_received == m_last_frame_header.framesize())) {
                        handleReceivedCompleteFrame();
                    }

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
