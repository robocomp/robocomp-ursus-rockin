/*
 * Copyright 2014 <copyright holder> <email>
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

#ifndef CURRENTTARGET_H
#define CURRENTTARGET_H
#include <QMutex>
#include <QMutexLocker>
#include <qmat/QMatAll>

class CurrentTarget
{
	public:
		CurrentTarget();					
		void reset(); 						
		QVec getTranslation() const;			
		void setTranslation(const QVec &t);
		QVec getRotation() const;		
		void setRotation(const QVec &r);
		bool isActive() const;
		void setActive(bool a);
		bool isWithoutPlan() const ;
		void setWithoutPlan(bool w); 
		void print();
		ulong getElapsedTime() const;  //ms
		bool hasRotation() const;
		void setHasRotation(bool a);
		enum class Command { GOTO, SETHEADING, STOP, CHANGETARGET};
		Command command;
		
	private:
		mutable QMutex mutex;
		QVec targetTr;
		QVec targetRot;
		bool active;
		bool withoutPlan;
		QTime reloj;
		bool doRotation;
		
};

#endif // CURRENTTARGET_H
