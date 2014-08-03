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

#include "currenttarget.h"

CurrentTarget::CurrentTarget()
{
	reset();
}

void CurrentTarget::reset()
{
	QMutexLocker  ml(&mutex);
	active = false; 
	withoutPlan = true; 
	targetTr = QVec::zeros(3); 
	targetRot = QVec::zeros(3);
	reloj.start();
}

QVec CurrentTarget::getTranslation() const
{
	QMutexLocker ml(&mutex);
	return targetTr;
}

void CurrentTarget::setTranslation(const QVec& t)
{
	QMutexLocker ml(&mutex);
	targetTr = t;
}

QVec CurrentTarget::getRotation() const
{
	QMutexLocker ml(&mutex);
	return targetRot;
}

void CurrentTarget::setRotation(const QVec& r)
{
	QMutexLocker ml(&mutex);
	targetRot = r;
}

bool CurrentTarget::isActive() const
{
	QMutexLocker ml(&mutex);
	return active;
}

void CurrentTarget::setActive(bool a)
{
	QMutexLocker ml(&mutex);
	active = a;
}
bool CurrentTarget::isWithoutPlan() const
{
	QMutexLocker ml(&mutex);
	return withoutPlan;
}
void CurrentTarget::setWithoutPlan(bool w)
{
	QMutexLocker ml(&mutex);
	withoutPlan = w;
}
void CurrentTarget::print()
{
	QMutexLocker ml(&mutex);
	qDebug() << "------------------------------------";
	qDebug() << "CurrentTarget  ---------------------";
	qDebug() << "	Active:" << active;
	qDebug() << "	Translation:" << targetTr;
	qDebug() << "	Rotation:" << targetRot;
	qDebug() << "	WithoutPlan" << withoutPlan;
	qDebug() << "	ElapsedTime" << reloj.elapsed();
	
	qDebug() << "------------------------------------";
}
