/*
 * Copyright 2014 pbustos <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <qmat/QMatAll>
#include <innermodel/innermodel.h>
#include <Laser.h>

class Localizer : public QObject
{
    Q_OBJECT

	public:
		Localizer(InnerModel *inner);
		void localize(const RoboCompLaser::TLaserData &laser, InnerModel *inner);
		

	private:
		InnerModel *clonModel;
		
		void renderLaser(const QVec &point, float alfa, const QVec &listaAngles);
		void recursiveIncludeMeshes(InnerModelNode *node, QString robotId, bool inside, std::vector<QString> &in, std::vector<QString> &out);
};
#endif // LOCALIZER_H