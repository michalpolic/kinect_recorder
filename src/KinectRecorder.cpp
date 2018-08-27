
#include <iostream>
#include <Windows.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include "gflags/gflags.h"

#include "io.hpp"

#ifndef LOAD_CMD_IO_FLAGS
#define LOAD_CMD_IO_FLAGS
DEFINE_string(d, "",
	"path to the root directory for capturing");
DEFINE_string(alg, "cudakde",
	"used algorithm for ir data unwrapping [cpu, cl, clkde, cuda, cudakde]");
#endif

using namespace kinectrec;

// Save the output of Kinect v2
int main(int argc, char *argv[])
{
	google::ParseCommandLineFlags(&argc, &argv, true);
	std::string base_path(FLAGS_d);
	if (base_path.size() == 0) {
		std::cerr << "It must be defined path where to save images (use -d=\"<path>\").";
		exit(0);
	}
	
	// init pipeline 
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	if (FLAGS_alg.compare("cpu"))
		pipeline = new libfreenect2::CpuPacketPipeline();
	if (FLAGS_alg.compare("cl"))
		pipeline = new libfreenect2::CpuPacketPipeline();
	if (FLAGS_alg.compare("clkde"))
		pipeline = new libfreenect2::CpuPacketPipeline();
	if (FLAGS_alg.compare("cuda"))
		pipeline = new libfreenect2::CpuPacketPipeline();
	if (FLAGS_alg.compare("cudakde"))
		pipeline = new libfreenect2::CpuPacketPipeline();
	if (!pipeline) {
		std::cerr << "Algorithm was not recognized (we recommend -alg=\"cudakde\").";
		exit(0);
	}

	// open device
	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	dev = freenect2.openDevice(serial, pipeline);
	
	// setup listeners
	int types = libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;
	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	
	// start listening
	if (!dev->start()) {
		std::cerr << "The listening doesn't start.\n";
		exit(0);
	}

	// registration setup
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4), bigdepth(1920, 1082, 4);

	// obtain and save images
	std::cout << "You can end the process by pressing the escape key.\n";
	while (true) {
		if (GetAsyncKeyState(VK_ESCAPE)) {
			std::cout << "The escape key was pressed.\n";
			break;
		}
		if(!listener.waitForNewFrame(frames, 5000)) {
			std::cerr << "None image was obtained in last 5 seconds.\n";
			break;
		}
		std::cout << "> new image was obtained \n";

		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered, true, &bigdepth);
		io::flipX(rgb);
		io::flipX(&bigdepth);

		// save rgb and registered depth images
		io::saveImgs(base_path, rgb, NULL, NULL, NULL, &bigdepth);

		listener.release(frames);
	}

	dev->stop();
	dev->close();
	delete registration;
}