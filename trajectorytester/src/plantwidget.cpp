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

#include "plantwidget.h"


PlantWidget::PlantWidget(QFrame *parent, QPointF pxCorner_, QPointF RXCorner_, QPointF pzCorner_, QPointF RZCorner_)
{
	pxCorner = pxCorner_;
	RXCorner = RXCorner_;
	pzCorner = pzCorner_;
	RZCorner = RZCorner_;
	
	intervals.append(QPair<QPointF,QPointF>(pxCorner,RXCorner)); 
	intervals.append(QPair<QPointF,QPointF>(pzCorner,RZCorner)); 
	m = QMat::afinTransformFromIntervals( intervals );
	
	intervalsI.append(QPair<QPointF,QPointF>(RXCorner, pxCorner)); 
	intervalsI.append(QPair<QPointF,QPointF>(RZCorner, pzCorner)); 
	mI = QMat::afinTransformFromIntervals( intervalsI );
	
	QGraphicsScene *gs = new QGraphicsScene();
	setScene(gs);
	QPixmap px("planta.png");
	px = px.scaled(size().width()-100,size().height()-20);
	this->scene()->addPixmap(px);
	setParent(parent);
	setMouseTracking(true);
	setFrameStyle(QFrame::Panel | QFrame::Sunken);
	
	QPixmap rob("ursusTop.png");
	rob = rob.scaled(20,20);
	robPtr =  gs->addPixmap(rob);
	qDebug() << pxCorner << pzCorner;
	robPtr->setRotation(-90);
	robPtr->moveBy(pxCorner.x()-20, pzCorner.x()-20);

}

void PlantWidget::moveRobot(float x, float y, float alfa)
{
	QVec p = mI * QVec::vec3(x, y, 1);
	//p.print("p");
	robPtr->setTransformOriginPoint(10,10);
	robPtr->setX( p.x());
	robPtr->setY( p.y());	
	robPtr->setRotation(alfa*180./M_PI - 90);
	update();
}

void PlantWidget::mouseMoveEvent(QMouseEvent *event)
{
	//qDebug() << event->pos();
	
	QVec p = m * QVec::vec3(event->x(), event->y(), 1);
	QVec res = QVec::vec3(p.x(), 0 , p.y());
	emit mouseMove(res);
	
}

void PlantWidget::mousePressEvent(QMouseEvent *event)
{
	//qDebug() << event->pos();
	QVec p = m * QVec::vec3(event->x(), event->y(), 1);
	QVec res = QVec::vec3(p.x(), 0 , p.y());
	emit mousePress(res);
	
}