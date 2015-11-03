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
	}
	
	
	bool logRGB(uint8_t *data, int32_t width, int32_t height, string frame_id="", int32_t seq=-1)
	{
		time_t sec;
		suseconds_t usec;
		std::string header = getHeaderFor("image", &sec, &usec);

		logFileMutex.lock();
		logFile << header;
		logFile << "  message:\n";
		logFile << "   header:\n";
		logFile << "     seq: " << std::to_string(seq);
		logFile << "     stamp:\n";
		logFile << "       secs: " << std::to_string(sec);
		logFile << "       nsecs: " << std::to_string(usec);
		logFile << "   x: " << float(x)/1000. << "\n";
		logFile << "   z: " << float(z)/1000. << "\n";
		logFile << "   theta: " << float(alpha) << "\n";
		logFileMutex.unlock();
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

