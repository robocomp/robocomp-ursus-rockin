/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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


#ifndef QLINE2D_H
#define QLINE2D_H

#include <QtCore>
#include <limits>
#include <qmat/qvec.h>

using namespace RMat;

class QLine2D : public QVec
{

	public:
		QLine2D();
		QLine2D(const QVec &p1, const QVec &p2);
		QLine2D(T x1, T y1, T x2, T y2);
		QLine2D(const QLine2D& other);
		QLine2D(const QVec &dirVector, T x, T y);
		virtual ~QLine2D(){};
		virtual QLine2D& operator=(const QLine2D& other);
		virtual bool operator==(const QLine2D& other) const;
		T perpendicularDistanceToPoint(const QVec &point);
		T signedAngleWithLine2D(const QLine2D &line);
		void print(const QString &p) {qDebug() << p << A << B << C;};
		float getA() const {return A;};
		float getB() const {return B;};
		float getC() const {return C;};
		QLine2D getPerpendicularLineThroughPoint(const QVec &point);  //2D point
		QVec getDirectionVector() const { return QVec::vec2(-B,A);};;
		QVec getPerpendicularVector() const { return QVec::vec2(A,B);};
		QVec intersectionPoint(const QLine2D &l);
		QLine2D getNormalLineThroughOrigin();
		QVec getIntersectionPointOfNormalThroughOrigin();
		QVec getNormalForOSGLineDraw();
		QLine2D getPlus45DegreesLinePassingThroughPoint(const QVec &point);
		T getAngleWithZAxis();
		QVec pointAlongLineStartingAtP1AtLanda(const QVec &p1, float landa);

		
	private:
		float A, B, C;
};

#endif // LINE2D_H
















