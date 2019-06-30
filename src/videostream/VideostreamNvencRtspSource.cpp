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
 * VideostreamNvencRtspSource component.
 *
 * @author Kevin Yu
 */

#ifdef HAVE_NVENC_RTSP

#include <memory>
#include <string>
#include <algorithm>

// Ubitrack
#include <utDataflow/Component.h>
#include <utDataflow/PushSupplier.h>
#include <utDataflow/ComponentFactory.h>

#include <utVision/Image.h>

#include "nvenc_rtsp/ClientPipeRTSP.h"

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

    class VideoFormatMap : public std::map<std::string, NvPipe_Format > {
    public:
        VideoFormatMap() {
            (*this)["RGBA32"] = NVPIPE_RGBA32; // depr. NVPIPE_BGRA32
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
        class NVencRtspSourceComponent : public Dataflow::Component
        {
            Dataflow::PushSupplier< Measurement::ImageMeasurement > m_outPort;

            std::shared_ptr<ClientPipeRTSP> m_videoPipe = nullptr;

            std::string m_rtspAddress;

            // nvenc video codec
            NvPipe_Codec m_video_codec;

            // nvenc video (pixel) format
            NvPipe_Format m_video_format;
        public:
            NVencRtspSourceComponent(const std::string& name, boost::shared_ptr< Graph::UTQLSubgraph > pConfig)
                : Dataflow::Component( name ),
                  m_outPort( "Output", *this ),
                  m_rtspAddress("rtsp://127.0.0.1:55555/live"),
                  m_video_codec(NVPIPE_H264),
                  m_video_format(NVPIPE_RGBA32)
            {

                // check for configurations and reset values if necessary
                pConfig->m_DataflowAttributes.getAttributeData( "rtspAddress", m_rtspAddress );

                if (pConfig->m_DataflowAttributes.hasAttribute("nvVideoCodec")) {
                    std::string sVideoCodec = pConfig->m_DataflowAttributes.getAttributeString("nvVideoCodec");
                    if (videoCodecMap.find(sVideoCodec) == videoCodecMap.end())
                        UBITRACK_THROW("unknown video codec: \"" + sVideoCodec + "\"");
                    m_video_codec = videoCodecMap[sVideoCodec];
                }

                if (pConfig->m_DataflowAttributes.hasAttribute("nvVideoFormat")) {
                    std::string sVideoFormat = pConfig->m_DataflowAttributes.getAttributeString("nvVideoFormat");
                    if (videoFormatMap.find(sVideoFormat) == videoFormatMap.end())
                        UBITRACK_THROW("unknown video format: \"" + sVideoFormat + "\"");
                    m_video_format = videoFormatMap[sVideoFormat];
                }
                m_videoPipe = nullptr;

            }

            ~NVencRtspSourceComponent()
            {
                if(m_videoPipe != nullptr) m_videoPipe->cleanUp();
                m_videoPipe = nullptr;
            }

            virtual void start()
            {

                if(m_videoPipe == nullptr)
                {
   
                    // registers callback when frame is received...
                    m_videoPipe = std::make_shared<ClientPipeRTSP>(m_rtspAddress, m_video_format, m_video_codec,
                         [&](cv::Mat mat, uint64_t timestamp) {

                            Vision::Image::ImageFormatProperties props;
                            size_t elementSize = 32;
                            switch (m_video_format) 
                            {
                                case NVPIPE_RGBA32:
                                    props.imageFormat = Vision::Image::RGBA;
                                    props.bitsPerPixel = 32;
                                    props.channels = 4;
                                    props.matType = CV_8UC4;
                                    props.depth = CV_8U;
                                    elementSize = 4;
                                    break;
                                // assume luminance for 8-bit 8UC1 and depth otherwise
                                case NVPIPE_UINT8:
                                    props.imageFormat = Vision::Image::LUMINANCE;
                                    props.bitsPerPixel = 8;
                                    props.channels = 1;
                                    props.matType = CV_8UC1;
                                    props.depth = CV_8U;
                                    elementSize = 1;
                                    break;

                                // how can we distinguish DEPTH from LUMINANCE Images here ??
                                // can we send metadata with the RTSP frames ??
                                case NVPIPE_UINT16:
                                    props.imageFormat = Vision::Image::DEPTH;
                                    props.bitsPerPixel = 16;
                                    props.channels = 1;
                                    props.matType = CV_16UC1;
                                    props.depth = CV_16U;
                                    elementSize = 2;
                                    break;
                                case NVPIPE_UINT32:
                                    props.imageFormat = Vision::Image::DEPTH;
                                    props.bitsPerPixel = 32;
                                    props.channels = 1;
                                    props.matType = CV_32SC1;
                                    props.depth = CV_32S;
                                    elementSize = 4;
                                    break;
                                default:
                                    std::cout << "Invalid image format" << std::endl;
                            }

                           Vision::Image::Ptr currentImage(new Vision::Image(mat, props));
                           m_outPort.send(Measurement::ImageMeasurement(timestamp, currentImage));
                         });
                }

            }
            virtual void stop()
            {
               if(m_videoPipe != nullptr) m_videoPipe->cleanUp();
               m_videoPipe = nullptr;
            }
    

        };

        UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) 
        {
            cf->registerComponent< NVencRtspSourceComponent > ( "VideostreamNvencRtspSource" );

        }
        
    }
}

#endif // HAVE_NVENC_RTSP