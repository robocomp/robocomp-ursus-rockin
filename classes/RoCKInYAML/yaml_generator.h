/*
 * Copyright 2015 Luis J. Manso
 *
 * This file is part of RoboComp-RoCKIn
 *
 * RoboComp-RoCKIn Converter is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * RoboComp-RoCKIn is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with RoboComp-RoCKIn.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <mutex>

#include <base64_rockin.hpp>


class RoCKInYAML
{
public:
	RoCKInYAML(std::string path)
	{
		logFile.open(path);
		if (not logFile.is_open())
		{
			throw std::string("RoCKInYAML: Can't open log file \"")+path+std::string("\"");
		}
	}
	
	
	~RoCKInYAML()
	{
		logFile.close();
	}
	
	bool logPose2D(float x, float z, float alpha)
	{
		std::string header = getHeaderFor("pose2d");

		logFileMutex.lock();
		logFile << header;
		logFile << "  message:\n";
		logFile << "   x: " << float(x)/1000. << "\n";
		logFile << "   z: " << float(z)/1000. << "\n";
		logFile << "   theta: " << float(alpha) << "\n";
		logFileMutex.unlock();
		
		return true;
	}
	
	bool logPose(float x, float y, float z, float ox, float oy, float oz, float ow)
	{
		std::string header = getHeaderFor("pose");

		logFileMutex.lock();
		logFile << header;
		logFile << "  message:\n";
		logFile << "   position:\n";
		logFile << "    x: " << x << "\n";
		logFile << "    y: " << y << "\n";
		logFile << "    z: " << z << "\n";
		logFile << "   orientation:\n";
		logFile << "    x: " << ox << "\n";
		logFile << "    y: " << oy << "\n";
		logFile << "    z: " << oz << "\n";
		logFile << "    w: " << ow << "\n";
		logFileMutex.unlock();
		
		return true;
	}
	
	bool logAudio(uint8_t *data, uint32_t length)
	{
		std::vector<uint8_t> tmp;
		tmp.resize(length);
		memcpy(&tmp[0], data, length);

		return logAudio(tmp);
	}
	bool logAudio(std::vector<uint8_t> data)
	{
		std::string header = getHeaderFor("audio");

		std::string odata;
		base64_encode(data, odata);

		logFileMutex.lock();
		logFile << header;
		logFile << "  message:\n";
		logFile << "   data: " << odata << "\n";
		logFileMutex.unlock();
		
		return true;
	}
	
	
	bool logRGB(uint8_t *data, int32_t width, int32_t height, string frame_id="", int32_t seq=-1)
	{
		time_t sec;
		suseconds_t usec;
		std::string header = getHeaderFor("image", &sec, &usec);
		
		std::string odata;
		std::vector<uint8_t> tmp;
		tmp.resize(width*height*3);
		memcpy(&tmp[0], data, width*height*3);
		base64_encode(tmp, odata);


		logFileMutex.lock();
		logFile << header;
		logFile << "  message:\n";
		logFile << "   header:\n";
		logFile << "     seq: " << std::to_string(seq) << "\n";
		logFile << "     stamp:\n";
		logFile << "       secs: " << std::to_string(sec) << "\n";
		logFile << "       nsecs: " << std::to_string(usec) << "\n";
		logFile << "     frame_id: '" << frame_id << "'\n";
		logFile << "   width: " << width << "\n";
		logFile << "   height: " << height << "\n";
		logFile << "   encoding: rbg8\n";
		logFile << "   is_bigendian: 0\n";
		logFile << "   step: " << std::to_string(width*3) << "\n";
		logFile << "   data: " << odata << "\n";
		logFileMutex.unlock();

		return true;
	}
	
	bool logCommand(std::string data)
	{
		std::string header = getHeaderFor("command");

		logFileMutex.lock();
		logFile << header;
		logFile << "  message:\n";
		logFile << "   data: '" << data << "'\n";
		logFileMutex.unlock();
		
		return true;
	}
	
private:
	
	std::string getHeaderFor(std::string topic, time_t *tv_sec=NULL, suseconds_t *tv_usec=NULL)
	{
		timeval tv;
		gettimeofday(&tv, NULL);
		if (tv_sec) *tv_sec = tv.tv_sec;
		if (tv_usec) *tv_usec = tv.tv_usec;

		std::string ret("- topic: ");
		ret += topic;
		ret += std::string("\n");
		ret += std::string("  secs: ")  + std::to_string(tv.tv_sec) + "\n";
		ret += std::string("  nsecs: ") + std::to_string(tv.tv_usec) + "000\n";
		
		return ret;
	}
	
	std::mutex logFileMutex;
	 ofstream logFile;
};

