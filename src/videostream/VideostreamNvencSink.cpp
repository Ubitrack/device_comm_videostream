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
 * VideostreamNvencSink component.
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

// Ubitrack
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/ComponentFactory.h>

#include <utVision/Image.h>

#include <NvPipe.h>

#include "VideostreamNvencProtocol.h"

namespace {
    class VideoCodecMap : public std::map<std::string, NvPipe_Codec > {
    public:
        VideoCodecMap() {
            (*this)["H264"] = NVPIPE_H264;
            (*this)["HEVC"] = NVPIPE_HEVC;
        }
    };
    static VideoCodecMap videoCodecMap;

    class VideoCompressionMap : public std::map<std::string, NvPipe_Compression > {
    public:
        VideoCompressionMap() {
            (*this)["LOSSLESS"] = NVPIPE_LOSSLESS;
            (*this)["LOSSY"] = NVPIPE_LOSSY;
        }
    };
    static VideoCompressionMap videoCompressionMap;

    class VideoFormatMap : public std::map<std::string, NvPipe_Format > {
    public:
        VideoFormatMap() {
            (*this)["BGRA32"] = NVPIPE_BGRA32;
            (*this)["UINT4"] = NVPIPE_UINT4;
            (*this)["UINT8"] = NVPIPE_UINT8;
            (*this)["UINT16"] = NVPIPE_UINT16;
            (*this)["UINT32"] = NVPIPE_UINT32;
        }
    };
    static VideoFormatMap videoFormatMap;
}


namespace Ubitrack { namespace Vision {

        static log4cpp::Category& logger( log4cpp::Category::getInstance( "Drivers.VideostreamNvencSink" ) );

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
 * - Ubitrack::Measurement::ImageMeasurement: NetworkVideostreamNvencSink
 */
        class VideostreamNvencSinkComponent
                : public Dataflow::Component
        {

            // consumer port
            Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;


            /// the boost asio underlying network service
            boost::asio::io_service m_ioService;

            /// network socket
            boost::shared_ptr< boost::asio::ip::udp::socket > m_socket;
            boost::shared_ptr< boost::asio::ip::udp::endpoint > m_endpoint;

            /// thread that runs the boost asio network service, (if connections was established)
            boost::scoped_ptr< boost::thread > m_pNetworkThread;

            /// ip-address trying to reach
            std::string m_dstAddress;

            /// port at destination trying to reach
            std::string m_dstPort;

            /// nvenc video encoder (NvPipe)
            boost::shared_ptr<NvPipe> m_video_encoder;

            // nvenc video codec
            NvPipe_Codec m_video_codec;

            // nvenc compression method
            NvPipe_Compression m_video_compression;

            // nvenc video (pixel) format
            NvPipe_Format m_video_format;

            // nvenc expected framerate
            unsigned int m_framerate;

            // nvenc expected bitrate
            unsigned int m_bitrate;

            // local send buffer
            std::vector<uint8_t> m_send_buffer;

        public:

            /** constructor */
            VideostreamNvencSinkComponent( const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig )
                    : Dataflow::Component( name )
                    , m_inPort( "Input", *this, boost::bind( &VideostreamNvencSinkComponent::eventIn, this, _1 ) )
                    , m_ioService()
                    , m_dstAddress( "127.0.0.1" )
                    , m_dstPort( "21844" ) // or 0x5554 as hex
                    , m_video_codec(NVPIPE_H264)
                    , m_video_compression(NVPIPE_LOSSLESS)
                    , m_video_format(NVPIPE_BGRA32)
                    , m_framerate(30)
                    , m_bitrate(20)
            {
                using boost::asio::ip::udp;

                // check for configurations and reset values if necessary
                pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_dstPort );
                if ( pConfig->m_DataflowAttributes.hasAttribute( "networkDestination" ) )
                {
                    m_dstAddress = pConfig->m_DataflowAttributes.getAttributeString( "networkDestination" );
                }
                if (pConfig->m_DataflowAttributes.hasAttribute("nvVideoCodec")) {
                    std::string sVideoCodec = pConfig->m_DataflowAttributes.getAttributeString("nvVideoCodec");
                    if (videoCodecMap.find(sVideoCodec) == videoCodecMap.end())
                        UBITRACK_THROW("unknown video codec: \"" + sVideoCodec + "\"");
                    m_video_codec = videoCodecMap[sVideoCodec];
                }

                if (pConfig->m_DataflowAttributes.hasAttribute("nvVideoCompression")) {
                    std::string sVideoCompression = pConfig->m_DataflowAttributes.getAttributeString("nvVideoCompression");
                    if (videoCompressionMap.find(sVideoCompression) == videoCompressionMap.end())
                        UBITRACK_THROW("unknown video compression: \"" + sVideoCompression + "\"");
                    m_video_compression = videoCompressionMap[sVideoCompression];
                }

                if (pConfig->m_DataflowAttributes.hasAttribute("nvVideoFormat")) {
                    std::string sVideoFormat = pConfig->m_DataflowAttributes.getAttributeString("nvVideoFormat");
                    if (videoFormatMap.find(sVideoFormat) == videoFormatMap.end())
                        UBITRACK_THROW("unknown video format: \"" + sVideoFormat + "\"");
                    m_video_format = videoFormatMap[sVideoFormat];
                }

                pConfig->m_DataflowAttributes.getAttributeData( "frameRate", m_framerate);
                pConfig->m_DataflowAttributes.getAttributeData( "bitRate", m_bitrate );


                // open new socket which we use for sending stuff
                m_socket = boost::shared_ptr< udp::socket >( new udp::socket (m_ioService) );
                m_socket->open( udp::v4() );

                // resolve destination pair and store the remote endpoint
                udp::resolver resolver( m_ioService );

                std::ostringstream portString;
                portString << m_dstPort;

                udp::resolver::query query( udp::v4(), m_dstAddress, portString.str() );
                m_endpoint = boost::shared_ptr< udp::endpoint >( new udp::endpoint( *resolver.resolve( query ) ) );

            }

        protected:

            void eventIn( const Measurement::ImageMeasurement& m )
            {

                if (!m_video_encoder) {
                    m_video_encoder.reset(NvPipe_CreateEncoder(m_video_format, m_video_codec, m_video_compression,
                                                               m_bitrate * 1000 * 1000, m_framerate));
                    if (!m_video_encoder) {
                        LOG4CPP_ERROR(logger, "Failed to create encoder: " << NvPipe_GetError(NULL));
                        return;
                    }
                }

                VideostreamNvencProtocol header;

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
                        cv::cvtColor(input_img, img, cv::COLOR_GRAY2BGRA);
                        break;
                    default:
                        img = input_img;
                }

                size_t raw_frame_size = img.total() * img.elemSize();
                size_t message_size_max = header.size() + raw_frame_size;

                // if send buffer size differs, re-initialize it to message_size_max (with 0)
                if (m_send_buffer.size() != message_size_max) {
                    m_send_buffer.clear();
                    m_send_buffer.resize(message_size_max, 0);
                }

                uint8_t* buffer_ptr = &m_send_buffer[header.size()];
                uint64_t size = NvPipe_Encode(m_video_encoder.get(), img.data, img.cols * img.elemSize(),
                                              buffer_ptr, raw_frame_size, m->width(), m->height(), false);
                if (0 == size) {
                    LOG4CPP_ERROR(logger, "Encode error: " << NvPipe_GetError(m_video_encoder.get()));
                    return;
                }

                // very simple transmission protocol:
                // header<int>(width, height, codec, format)
                // then encoded framedata from encoder

                header.timestamp(m.time()); // image timestamp
                header.width((unsigned short)m->width()); // image width
                header.height((unsigned short)m->height()); // image height
                header.codec(m_video_codec); // image codec H264/HEVC
                header.format(m_video_format); // image format BGRA32/UINT4/UINT8/UINT16/UINT32
                header.framesize(size); // size of resulting encoded frame
                header.copy_to(m_send_buffer);

                size_t actual_packet_size = header.size() + size;
                // assert that this fits into m_send_buffer !!

                LOG4CPP_TRACE( logger, "Sending image data to \"" << m_dstAddress << ":" << m_dstPort << "\"." );

                m_socket->send_to( boost::asio::buffer( m_send_buffer.data(), actual_packet_size ), *m_endpoint );
            }
        };

// register module at factory
        UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
            cf->registerComponent< VideostreamNvencSinkComponent > ( "VideostreamNvencSink" );

        }

    } } // namespace Ubitrack::Drivers
