/*
 *    Copyright (C) 2015 by YOUR NAME HERE
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
#include "specificworker.h"

void SpecificWorker::compute()
{
	qDebug()<<"numberOfSymbols "<<worldModel->numberOfSymbols();


	if (worldModel->numberOfSymbols()>0)
	{
		AGMModel::SPtr newModel(new AGMModel(worldModel));
		for (auto p : innerModelInfoVector)
		{
			AgmInner::includeInnerModel(newModel, p.second.toInt(), p.first);
		}
		newModel->save("agmModel.xml");
		AgmInner::extractInnerModel(newModel, "world")->save("extractInnerModel.xml");
		sendModificationProposal(worldModel, newModel);
		qFatal("The job was done. Exiting...");
	}
	
}


/**
 * @brief Search the imName of the innermodel node,(the name is the unique key for innerModel ), inside de AGM Model. The innermodel id is stored in the attribute "imName" of each symbol. 
 * It is found, return the id of the symbol, the unique key for AGMSymbols, otherwise returns -1.
 * 
 * @param n value of the attribute field imName...
 * @return symbol ID, -1 if it is not found
 */
int SpecificWorker::findName(QString n)
{
	for (uint32_t i=0; i<worldModel->symbols.size(); ++i)
	{	
		if (worldModel->symbols[i]->attributes.find("imName") != worldModel->symbols[i]->attributes.end() )
		{
			if (worldModel->symbols[i]->attributes["imName"] == n.toStdString() )
			{
// 				qDebug()<<"findName: FOUND"<<n<<worldModel->symbols[i]->identifier;
				return worldModel->symbols[i]->identifier;
			}
		}
	}	
// 	qDebug()<<"findName: NO ENCONTRADO"<<n<<-1;
	return -1;
}



void SpecificWorker::innerToAGM(InnerModelNode* node, int &symbolID, QList<QString>  lNode)
{
	QList<InnerModelNode*>::iterator i;	
	int p=symbolID;
	
	for (i=node->children.begin(); i!=node->children.end(); i++)
	{
		if ( !lNode.contains((*i)->id) )
		{	
			//Search name (key) of the innerModel node in AGM
			int existingID = findName ( (*i)->id ) ;
			if ( existingID == -1 )
			{	
				qDebug()<<node->id<<"link"<<(*i)->id;
				//symbol
				AGMModelSymbol::SPtr newSym = ImNodeToSymbol((*i));
								
				//edge 
				std::map<std::string, std::string> linkAttrs;

				linkAttrs.insert ( std::pair<std::string,std::string>("tx",float2str((*i)->getTr().x())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("ty",float2str((*i)->getTr().y())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("tz",float2str((*i)->getTr().z())) );
				linkAttrs.insert ( std::pair<std::string,std::string>("rx",float2str((*i)->getRxValue())));
				linkAttrs.insert ( std::pair<std::string,std::string>("ry",float2str((*i)->getRyValue())));
				linkAttrs.insert ( std::pair<std::string,std::string>("rz",float2str((*i)->getRzValue())));
				int32_t id = newSym->identifier;
				worldModel->addEdgeByIdentifiers(p,id,"RT",linkAttrs);
				innerToAGM((*i),id,lNode);
			}
			else
				innerToAGM((*i),existingID,lNode);
		}
	}	
}

AGMModelSymbol::SPtr SpecificWorker::ImNodeToSymbol(InnerModelNode* node)
{
	std::map<std::string, std::string> attrs;
	attrs.insert ( std::pair<std::string,std::string>("imName",node->id.toStdString()) );
	
	AGMModelSymbol::SPtr newSym;
	
	//TODO innerModelNode cast to the type and translate the name.
	// attribute "type" = mesh,plane,joint..
	string type;
	if (dynamic_cast<InnerModelJoint*>(node) != NULL)
	{
		InnerModelJoint *joint = dynamic_cast<InnerModelJoint*>(node);
		//QString id_, float lx_, float ly_, float lz_, float hx_, float hy_, float hz_, float tx_, float ty_, float tz_, float rx_, ;
		//float ry_, float rz_, float min_, float max_, uint32_t port_, std::string axis_, float home_, 
// 		attrs.insert ( std::pair<std::string,std::string>("lx",float2str(0.) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("ly",float2str(0.) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("lz",float2str(0.) ) );
// 		
// 		attrs.insert ( std::pair<std::string,std::string>("hx",float2str(joint->backhX) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("hy",float2str(joint->backhY) ) );
// 		attrs.insert ( std::pair<std::string,std::string>("hz",float2str(joint->backhZ) ) );
		
		attrs.insert ( std::pair<std::string,std::string>("min",float2str(joint->min) ) );
		attrs.insert ( std::pair<std::string,std::string>("max",float2str(joint->max) ) );
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(joint->port) ) );
		attrs.insert ( std::pair<std::string,std::string>("axis",joint->axis) );
		attrs.insert ( std::pair<std::string,std::string>("home",float2str(joint->home)) );
		type= "joint";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelPrismaticJoint*>(node) != NULL)
	{
		InnerModelPrismaticJoint *p = dynamic_cast<InnerModelPrismaticJoint*>(node);
// 	float value, offset;
// 	float min, max;
// 	float home;
// 	uint32_t port;
// 	std::string axis;
	
		attrs.insert ( std::pair<std::string,std::string>("value",float2str(p->value) ) );
		attrs.insert ( std::pair<std::string,std::string>("offset",float2str(p->offset) ) );
		attrs.insert ( std::pair<std::string,std::string>("min",float2str(p->min) ) );
		attrs.insert ( std::pair<std::string,std::string>("max",float2str(p->max) ) );
		attrs.insert ( std::pair<std::string,std::string>("home",float2str(p->home)) );
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(p->port)) );
		attrs.insert ( std::pair<std::string,std::string>("axis",p->axis) );
		
		type= "prismaticJoint";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	
	else if (dynamic_cast<InnerModelTouchSensor*>(node) != NULL)
	{
		InnerModelTouchSensor *touch = dynamic_cast<InnerModelTouchSensor*>(node);
// 		float nx, ny, nz;
		attrs.insert ( std::pair<std::string,std::string>("nx",float2str(touch->nx) ));
		attrs.insert ( std::pair<std::string,std::string>("ny",float2str(touch->ny)) );
		attrs.insert ( std::pair<std::string,std::string>("nz",float2str(touch->nz)) );
		
		//float min, max;// 	float value;
		attrs.insert ( std::pair<std::string,std::string>("min",float2str(touch->min) ));
		attrs.insert ( std::pair<std::string,std::string>("max",float2str(touch->max)) );
		attrs.insert ( std::pair<std::string,std::string>("value",float2str(touch->value)) );
	// 	uint32_t port;
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(touch->port)) );
		// QString stype;
		attrs.insert ( std::pair<std::string,std::string>("stype",touch->stype.toStdString()) );
		type = "touchSensor";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelDifferentialRobot*>(node) != NULL)
	{
		InnerModelDifferentialRobot *diff = dynamic_cast<InnerModelDifferentialRobot*>(node);
		//uint32_t port;
		//float noise;
		//bool collide;
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(diff->port)) );
		attrs.insert ( std::pair<std::string,std::string>("noise",float2str(diff->noise)) );
		
		string v="false";
		if (diff->collide)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collide",v) );
		
		type = "differentialRobot";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelOmniRobot*>(node) != NULL)
	{
		InnerModelOmniRobot *omni = dynamic_cast<InnerModelOmniRobot*>(node);
		//uint32_t port;
		//float noise;
		//bool collide;
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(omni->port)) );
		attrs.insert ( std::pair<std::string,std::string>("noise",float2str(omni->noise)) );
		
		string v="false";
		if (omni->collide)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collide",v) );
				
		type = "omniRobot";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelPlane*>(node) != NULL)
	{
		//QString id, InnerModelNode *parent, QString texture, float width, float height, float depth, int repeat, 
		//float nx, float ny, float nz, float px, float py, float pz, bool collidable)

		InnerModelPlane* plane = dynamic_cast<InnerModelPlane*>(node);	
		
		attrs.insert ( std::pair<std::string,std::string>("width",float2str(plane->width) ));
		attrs.insert ( std::pair<std::string,std::string>("height",float2str(plane->height)) );
		attrs.insert ( std::pair<std::string,std::string>("depth",float2str(plane->depth)) );
		
		attrs.insert ( std::pair<std::string,std::string>("nx",float2str(plane->normal.x()) ));
		attrs.insert ( std::pair<std::string,std::string>("ny",float2str(plane->normal.y()) ));
		attrs.insert ( std::pair<std::string,std::string>("nz",float2str(plane->normal.z()) ));
		
		attrs.insert ( std::pair<std::string,std::string>("px",float2str(plane->point.x()) ));
		attrs.insert ( std::pair<std::string,std::string>("py",float2str(plane->point.y()) ));
		attrs.insert ( std::pair<std::string,std::string>("pz",float2str(plane->point.z()) ));
		
		string v="false";
		if (plane->collidable)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collidable",v) );
		attrs.insert ( std::pair<std::string,std::string>("repeat",int2str(plane->repeat)) );
		attrs.insert ( std::pair<std::string,std::string>("texture",plane->texture.toStdString()) );
		type = "plane";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
		
	}
	else if (dynamic_cast<InnerModelRGBD*>(node) != NULL)
	{
		InnerModelRGBD* rgbd = dynamic_cast<InnerModelRGBD*>(node);		
		
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(rgbd->port)) );
		attrs.insert ( std::pair<std::string,std::string>("noise",float2str(rgbd->noise) ) );
		attrs.insert ( std::pair<std::string,std::string>("focal",float2str(rgbd->focal) ) );
		attrs.insert ( std::pair<std::string,std::string>("height",float2str(rgbd->height) ) );
		attrs.insert ( std::pair<std::string,std::string>("width",float2str(rgbd->width) ) );
		attrs.insert ( std::pair<std::string,std::string>("ifconfig",rgbd->ifconfig.toStdString()) );
		type ="rgbd";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelCamera*>(node) != NULL)
	{
		InnerModelCamera* cam = dynamic_cast<InnerModelCamera*>(node);		
		
		attrs.insert ( std::pair<std::string,std::string>("focal",float2str(cam->focal) ) );
		attrs.insert ( std::pair<std::string,std::string>("height",float2str(cam->height) ) );
		attrs.insert ( std::pair<std::string,std::string>("width",float2str(cam->width) ) );
		
		type = "camera";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelIMU*>(node) != NULL)
	{
		InnerModelIMU* imu = dynamic_cast<InnerModelIMU*>(node);		
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(imu->port)) );
		type = "imu";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelLaser*>(node) != NULL)
	{
		InnerModelLaser* laser = dynamic_cast<InnerModelLaser*>(node);
		//public:	uint32_t port;	uint32_t min, max;	float angle;	uint32_t measures;	QString ifconfig;
		
		attrs.insert ( std::pair<std::string,std::string>("port",int2str(laser->port)) );
		attrs.insert ( std::pair<std::string,std::string>("min",int2str(laser->min)) );
		attrs.insert ( std::pair<std::string,std::string>("max",int2str(laser->max)) );
		attrs.insert ( std::pair<std::string,std::string>("measures",int2str(laser->measures)) );
		attrs.insert ( std::pair<std::string,std::string>("angle",float2str(laser->angle)) );
		attrs.insert ( std::pair<std::string,std::string>("ifconfig",laser->ifconfig.toStdString()) );
		type = "laser";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelMesh*>(node) != NULL)
	{
		InnerModelMesh* mesh = dynamic_cast<InnerModelMesh*>(node);
		attrs.insert ( std::pair<std::string,std::string>("path",mesh->meshPath.toStdString()) );
		attrs.insert ( std::pair<std::string,std::string>("scalex",float2str(mesh->scalex)) );
		attrs.insert ( std::pair<std::string,std::string>("scaley",float2str(mesh->scaley)) );
		attrs.insert ( std::pair<std::string,std::string>("scalez",float2str(mesh->scalez)) );
		string v="false";
		if (mesh->collidable)
			v="true";
		attrs.insert ( std::pair<std::string,std::string>("collidable",v) );
		
		//enum RenderingModes { NormalRendering=0, WireframeRendering=1};
		v="NormalRendering";
		if (mesh->render==1)
			v="WireframeRendering";
		attrs.insert ( std::pair<std::string,std::string>("render",v) );
		
		type = "mesh";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelPointCloud*>(node) != NULL)
	{
		type = "pointCloud";
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	else if (dynamic_cast<InnerModelTransform*>(node) != NULL)
	{
		InnerModelTransform *t = dynamic_cast<InnerModelTransform*>(node);
		type="transform";
		attrs.insert ( std::pair<std::string,std::string>("engine",t->engine.toStdString()) );
		attrs.insert ( std::pair<std::string,std::string>("mass",float2str(t->mass)) );
		attrs.insert ( std::pair<std::string,std::string>("imType",type ) );
	}
	
	else
	{
		string err;
		err = "error: Type of node " + node->id.toStdString() + " is unknown.";
		throw err;
	}

	int32_t id =worldModel->getNewId();
	return worldModel->newSymbol(id,type,attrs);

}



// COMMON METHODS


/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	active = false;
	worldModel = AGMModel::SPtr(new AGMModel());
	worldModel->name = "worldModel";	
	
	timer.start(33);
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	for (auto i : params)
		printf("%s\n", i.first.c_str());

	
	
printf("%s: %d\n", __FILE__, __LINE__);
	
	RoboCompCommonBehavior::Parameter par;
	try
	{
printf("%s: %d\n", __FILE__, __LINE__);
		 par = params.at("AGMInnerAgent.InnerModels");
printf("%s: %d\n", __FILE__, __LINE__);
	}
	catch(std::exception e)
	{
		qFatal("Error reading config params: %s\n", e.what());
	}

printf("%s: %d\n", __FILE__, __LINE__);
	for (auto s : QString::fromStdString(par.value).split(";"))
	{
printf("%s: %d\n", __FILE__, __LINE__);	
		auto v = s.split(",");
		if( QFile(v[0]).exists() == true)
		{
printf("%s\n%s: %d\n", par.value.c_str(), __FILE__, __LINE__);
			InnerModel *innerModel = new InnerModel(v[0].toStdString());
printf("%s: %d\n", __FILE__, __LINE__);
			innerModelInfoVector.push_back(std::pair<InnerModel *, QString>(innerModel, v[1]));
printf("%s: %d\n", __FILE__, __LINE__);
		}
		else
		{
			qFatal("File %s specifed in config file not found: Exiting now.", v[0].toStdString().c_str());
		}
	}
	

	return true;
}



// ****** AGENTS *******
bool SpecificWorker::reloadConfigAgent()
{
	return true;
}

bool SpecificWorker::activateAgent(const ParameterMap &prs)
{
	bool activated = false;
	if (setParametersAndPossibleActivation(prs, activated))
	{
		if (not activated)
		{
			return activate(p);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool SpecificWorker::setAgentParameters(const ParameterMap &prs)
{
	bool activated = false;
	return setParametersAndPossibleActivation(prs, activated);
}

ParameterMap SpecificWorker::getAgentParameters()
{
	return params;
}

void SpecificWorker::killAgent()
{

}

int SpecificWorker::uptimeAgent()
{
	return 0;
}

bool SpecificWorker::deactivateAgent()
{
	return deactivate();
}

StateStruct SpecificWorker::getAgentState()
{
	StateStruct s;
	if (isActive())
	{
		s.state = Running;
	}
	else
	{
		s.state = Stopped;
	}
	s.info = p.action.name;
	return s;
}

void SpecificWorker::structuralChange(const RoboCompAGMWorldModel::Event &modification)
{
	mutex->lock();
	printf("structural change\n");
 	AGMModelConverter::fromIceToInternal(modification.newModel, worldModel);
 	mutex->unlock();
}


void SpecificWorker::edgesUpdated(const RoboCompAGMWorldModel::EdgeSequence &modifications)
{
	QMutexLocker lockIM(mutex);
	for (auto modification : modifications)
	{
		AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
	}
}

void SpecificWorker::edgeUpdated(const RoboCompAGMWorldModel::Edge &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	mutex->unlock();
}

void SpecificWorker::symbolUpdated(const RoboCompAGMWorldModel::Node &modification)
{
	mutex->lock();
 	AGMModelConverter::includeIceModificationInInternalModel(modification, worldModel);
 	mutex->unlock();
}



bool SpecificWorker::setParametersAndPossibleActivation(const ParameterMap &prs, bool &reactivated)
{
	printf("<<< setParametersAndPossibleActivation\n");
	// We didn't reactivate the component
	reactivated = false;

	// Update parameters
	params.clear();
	for (ParameterMap::const_iterator it=prs.begin(); it!=prs.end(); it++)
	{
		params[it->first] = it->second;
	}

	try
	{
		action = params["action"].value;
		std::transform(action.begin(), action.end(), action.begin(), ::tolower);
		//TYPE YOUR ACTION NAME
		if (action == "actionname")
		{
			active = true;
		}
		else
		{
			active = true;
		}
	}
	catch (...)
	{
		printf("exception in setParametersAndPossibleActivation %d\n", __LINE__);
		return false;
	}

	// Check if we should reactivate the component
	if (active)
	{
		active = true;
		reactivated = true;
	}

	printf("setParametersAndPossibleActivation >>>\n");

	return true;
}


void SpecificWorker::sendModificationProposal(AGMModel::SPtr &worldModel, AGMModel::SPtr &newModel)
{
	try
	{
		AGMMisc::publishModification(newModel, agmagenttopic_proxy, worldModel, "agmInnerCompAgent");
	}
	catch(...)
	{
		exit(1);
	}
}





