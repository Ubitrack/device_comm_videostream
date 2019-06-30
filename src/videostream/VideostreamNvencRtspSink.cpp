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
 * VideostreamNvencRtspSink component.
 *
 * @author Kevin Yu
 */

#ifdef HAVE_NVENC_RTSP

#include <memory>
#include <string>
#include <algorithm>

// Ubitrack
#include <utDataflow/Component.h>
#include <utDataflow/PushConsumer.h>
#include <utDataflow/ComponentFactory.h>

#include <utVision/Image.h>

#include "nvenc_rtsp/ServerPipeRTSP.h"

using namespace cv;
using namespace nvenc_rtsp;

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
            (*this)["RGBA32"] = NVPIPE_RGBA32;
            (*this)["UINT4"] = NVPIPE_UINT4;
            (*this)["UINT8"] = NVPIPE_UINT8;
            (*this)["UINT16"] = NVPIPE_UINT16;
            (*this)["UINT32"] = NVPIPE_UINT32;
        }
    };
    static VideoFormatMap videoFormatMap;
}

namespace Ubitrack 
{ 
    namespace Vision 
    {
        class NVencRtspSinkComponent : public Dataflow::Component
        {
            Dataflow::PushConsumer< Measurement::ImageMeasurement > m_inPort;

            std::shared_ptr<ServerPipeRTSP> m_videoPipe = nullptr;

            int m_dstPort;

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
        
        public:
            NVencRtspSinkComponent(const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig)
                : Dataflow::Component( name ),
                  m_inPort( "Input", *this, boost::bind( &NVencRtspSinkComponent::eventIn, this, _1 ) ),
                  m_dstPort( 55555 ),
                  m_video_codec(NVPIPE_H264),
                  m_video_compression(NVPIPE_LOSSLESS),
                  m_video_format(NVPIPE_RGBA32),
                  m_framerate(60),
                  m_bitrate(32)
            {
                // check for configurations and reset values if necessary
                pConfig->m_DataflowAttributes.getAttributeData( "networkPort", m_dstPort );

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

                m_videoPipe = std::make_shared<ServerPipeRTSP>(m_dstPort, m_video_format, m_video_compression, m_video_codec, m_bitrate, m_framerate);
            }

            ~NVencRtspSinkComponent()
            {
                if(m_videoPipe != nullptr) m_videoPipe->cleanUp();
            }

            void eventIn( const Measurement::ImageMeasurement& m )
            {

                cv::Mat input_img = m->Mat();
                Vision::Image::ImageFormatProperties img_props;
                m->getFormatProperties(img_props);

                int int_type = img_props.bitsPerPixel == 8 ? CV_8U : (img_props.bitsPerPixel == 16 ? CV_16U : CV_32S);

                // XXX this part is using CPU to convert image data - can we do it on GPU (CUDA) and then hand over a GPU Buffer ??
                cv::Mat img;
                switch(img_props.imageFormat) {
                    case Vision::Image::BGR:
                        cv::cvtColor(input_img, img, cv::COLOR_BGR2RGBA);
                        break;
                    case Vision::Image::RGB:
                        cv::cvtColor(input_img, img, cv::COLOR_RGB2RGBA);
                        break;
                    case Vision::Image::BGRA:
                        cv::cvtColor(input_img, img, cv::COLOR_BGRA2RGBA);
                        break;
                    case Vision::Image::LUMINANCE:
                    case Vision::Image::DEPTH:
                        input_img.convertTo(img, int_type);
                        break;
                    default:
                        // XXX should check for RGBA and complain otherwise !!
                        img = input_img;
                }

                // XXXX we need to pass the timestamp (m.time()) to the nvenc_rtsp library (timestamp is of type: unsigned long long)
                m_videoPipe->send_frame(img);

            }

        };

        UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
        {
            cf->registerComponent< NVencRtspSinkComponent > ( "VideostreamNvencRtspSink" );

        }
        
    }
}

#endif // HAVE_NVENC_RTSP