/*
 *    Copyright (C) 2008-2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
/** \mainpage RoboComp::cuba2dnaturallandmarksComp
 *
 * \section intro_sec Introduction
 *
 * CubaFeaturesComp provides laser features. Transform laser distances to points, segments or circles.
 *
 * \section interface_sec Interface
 *
 * 
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * cuba2dnaturallandmarks does not have specific software dependencies.
 *
 * \subsection install2_ssec Compile
 * cd Components/Robolab/Stable/cuba2dnaturallandmarksComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * sudo make install
 *
 * \subsection install2_ssec Compile and install
 * cd Components/Robolab/Stable/cuba2dnaturallandmarksComp
 *
 *
 * \section guide_sec User guide
 *
 *
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * 
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/cuba2dnaturallandmarksComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * 
 *
 */
// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>

// View the config.h file for config options like
// enable/disable server, QtGui, etc...
#include "config.h"

#include "worker.h"
#ifdef SERVER
// Interface implementation
#include "cuba2dnaturallandmarksI.h"
#endif

// Includes for remote proxy example
// #include <Remote.h>
#include <CCDAmatching.h>
#include <Laser.h>
#include <DifferentialRobot.h>

// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCCDAmatching;
using namespace RoboCompCuba2Dnaturallandmarks;
using namespace RoboCompLaser;
using namespace RoboCompDifferentialRobot;

class Cuba2DnaturallandmarksComp : public RoboComp::Application
{
private:
	// User private data here

	void initialize();

public:
	virtual int run(int, char*[]);
};

void Cuba2DnaturallandmarksComp::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int Cuba2DnaturallandmarksComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif
	int status=EXIT_SUCCESS;

	// Remote server proxy access example
	// RemoteComponentPrx remotecomponent_proxy;
	LaserPrx laser_proxy;
	DifferentialRobotPrx base_proxy;

	string proxy;

	// User variables

	initialize();

	try
	{
		// Load the remote server proxy
		proxy = communicator()->getProperties()->getProperty("LaserProxy");
		cout << "[" << PROGRAM_NAME << "]: Loading [" << proxy << "] proxy at (" << "LaserProxy" << ")..." << endl;
		if( proxy.empty() )
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy config! Check config file for missing of incorrect proxies" << endl;
			return EXIT_FAILURE;
		}
		laser_proxy = LaserPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
		if( !laser_proxy )
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy!" << endl;
			return EXIT_FAILURE;		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex << endl;
		return EXIT_FAILURE;
	}
	cout << "LaserProxy initialized Ok!" << endl;

	///Base Proxy
	try
	{
		// Load the remote server proxy
		proxy = communicator()->getProperties()->getProperty("DifferentialRobotProxy");
		cout << "[" << PROGRAM_NAME << "]: Loading [" << proxy << "] proxy at (" << "DifferentialRobotProxy" << ")..." << endl;
		if( proxy.empty() )
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy config! Check config file for missing of incorrect proxies" << endl;
			return EXIT_FAILURE;
		}
		base_proxy = DifferentialRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
		if( !base_proxy )
		{
			cout << "[" << PROGRAM_NAME << "]: Error loading proxy!" << endl;
			return EXIT_FAILURE;		}
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex << endl;
		return EXIT_FAILURE;
	}
	cout << "BaseProxy initialized Ok!" << endl;


	float h = atof(communicator()->getProperties()->getProperty("Height").c_str());
	Worker *worker = new Worker(laser_proxy, base_proxy, h);

	try
	{
#ifdef SERVER
		// Server adapter creation and publication
		Ice::ObjectAdapterPtr adapter = communicator()->createObjectAdapter("Cuba2DnaturallandmarksComp");
		Cuba2DnaturallandmarksI *cuba2dnaturallandmarksI = new Cuba2DnaturallandmarksI(worker );
		adapter->add(cuba2dnaturallandmarksI, communicator()->stringToIdentity("cuba2dnaturallandmarks"));
		adapter->activate();

		cout << SERVER_FULL_NAME " started" << endl;
#endif

		// User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
		// Run QT Application Event Loop
		ignoreInterrupt(); // Ignore the ctrl-c on console
		a.setQuitOnLastWindowClosed( true );
		a.exec();
#else
		a.exec();
#endif
		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

#ifdef USE_QTGUI
		a.quit();
#endif
	}

	return status;
}

int main(int argc, char* argv[])
{
	bool hasConfig = false;
	string arg;
	Cuba2DnaturallandmarksComp app;

	// Search in argument list for --Ice.Config= argument
	for (int i = 1; i < argc; ++i)
	{
		arg = argv[i];
		if ( arg.find ( "--Ice.Config=", 0 ) != string::npos )
			hasConfig = true;
	}

	if ( hasConfig )
		return app.main( argc, argv );
	else
		return app.main(argc, argv, "config"); // "config" is the default config file name
}
