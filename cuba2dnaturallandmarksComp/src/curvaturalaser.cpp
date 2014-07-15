/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
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
#include "curvaturalaser.h"


CURVATURALASER::CURVATURALASER
(int TamanoLaser_, int K_, int KMAX_, int KMIN_, double UmbralAdapt_,double UmbralEsquina_)
{
	int N_EsquinasMax=100;
	
	TamanoLaser=TamanoLaser_;
	K=K_;

	KMax=KMAX_;			/* Valor Maximo de K */
	KMin=KMIN_;			/* Valor Minimo de K */

	UmbralEsquina=UmbralEsquina_;	        /* Umbral deteccion esquinas */
	UmbralAdapt=UmbralAdapt_;				/* Umbral calculo curvatura adaptativa */

	/* Creamos espacio para ArrayIN, Curvatura y CurvDef */

	ArrayIN=new DatoLASER[TamanoLaser];	/* Array de datos laser de entrada 	   */
	Curvatura=new double[2*TamanoLaser];	/* Array de datos de curvatura 		   */
	CurvDef= new double[2*TamanoLaser];	/* Array de datos de curvatura definitivos */
	Esquinas=new double[6*N_EsquinasMax];/* Array de datos de esquinas */
	Circles=new double[4];
}

CURVATURALASER::~CURVATURALASER() // (added by Ricardo Vazquez)
{
	/* Borrar ArrayIN y Curvatura */
	delete[](ArrayIN);	
	delete[](Curvatura);	
	delete[](CurvDef);
	delete[](Esquinas);
	delete[](Circles);
}

inline double CURVATURALASER::DistEuclidea(double a, double b, double a1, double b1)
{
	return ((double)sqrt((double)((a-a1)*(a-a1)+(b-b1)*(b-b1))));
}

int CURVATURALASER::CalculoCurvatura(int grupo)
{
	int error=0;

	if (K>KMax) 
		{
		std::cout << "Error:: K excesivamente grande\n";
		exit(-1);	/* error=1 */
		}
	if (K<=0)	/* Calculo curvatura adaptativo 	*/
		{
		error=ObtenerCurvaturaAdaptativo(grupo);
		}
	else		/* Calculo curvatura k constante 	*/
		{
		error=ObtenerCurvaturaKcte();
		}
	return(error);
}

inline double CURVATURALASER::CurvLocal(double f0, double f1, double b0, double b1)
{
	double r, signo;
	double modulo_f;
	double modulo_b;
	double cos_angulofb;

	// Formula de Rosenfeld y Johnston para la curvatura
	modulo_f=(double)sqrt((double)(f0*f0+f1*f1));
	modulo_b=(double)sqrt((double)(b0*b0+b1*b1));
	cos_angulofb=((double)(f0*b0+f1*b1))/(modulo_f*modulo_b);
	r=fabs(0.5+0.5*cos_angulofb);

	// Mejorada por Pedro Reche
	signo=f0*b1>b0*f1 ? (1.0) :(-1.0);

	return(signo*r);
}

int CURVATURALASER::ObtenerCurvaturaAdaptativo(int grupo)
{
	register int i;
	int cont=0;
	int error=0;
	int kdAhora;
	int kaAhora;
	double f0,f1,b0,b1;
	double distkdAhora;	/* Distancia pixel a pixel hacia delante */
	double distkaAhora;	/* Distancia pixel a pixel hacia atras */
	int signo;

// 	if (TamanoLaser<2*KMin)
// 		{
// 		std::cout << "TamanoLaser muy pequenno \n";
// 	
// 		}
	// Inicializacion: Calculo de kaAhora y distkaAhora (kdAhora=KMin) 
	// para el primer punto (i=KMin)
	kdAhora=KMin;
	kaAhora=KMin;
	distkaAhora=0.0;
	distkdAhora=0.0;

	for (i=KMin;i>0;i--)
		distkaAhora=distkaAhora+DistEuclidea(ArrayIN[i].f,ArrayIN[i].c,ArrayIN[i-1].f,ArrayIN[i-1].c);
	for (i=KMin;i<KMin+KMin;i++)
		distkdAhora=distkdAhora+DistEuclidea(ArrayIN[i].f,ArrayIN[i].c,ArrayIN[i+1].f,ArrayIN[i+1].c);
 
	do
		{
		kdAhora++;
		distkdAhora=distkdAhora
			+DistEuclidea(ArrayIN[KMin+kdAhora-1].f,ArrayIN[KMin+kdAhora-1].c,
						  ArrayIN[KMin+kdAhora].f,ArrayIN[KMin+kdAhora].c);
		}
	while ((DistEuclidea(ArrayIN[KMin].f,ArrayIN[KMin].c,
						 ArrayIN[KMin+kdAhora].f,ArrayIN[KMin+kdAhora].c)+UmbralAdapt)>distkdAhora 
			&& kdAhora<KMax);
	kdAhora--;
	distkdAhora=distkdAhora
		-DistEuclidea(ArrayIN[KMin+kdAhora].f,ArrayIN[KMin+kdAhora].c,
					  ArrayIN[KMin+kdAhora+1].f,ArrayIN[KMin+kdAhora+1].c);
  
	// Bucle principal:	- podemos hacer kaAhora mayor, pero sin que (i-kaAhora) sea menor que 0
	//					- podemos hacer kdAhora mayor, pero sin que (i+kdAhora) sea mayor que TamanoLaser
	for (i=KMin;i<TamanoLaser-KMin;i++)
		{
		// Actualizacion de los campos ka y kd de ArrayIN y calculo de curvatura(cont) 
		// en funcion de kaAhora y kdAhora
		ArrayIN[i].ka=kaAhora;
		ArrayIN[i].kd=kdAhora;
		f0=ArrayIN[i+kdAhora].f-ArrayIN[i].f;
		f1=ArrayIN[i+kdAhora].c-ArrayIN[i].c;
		b0=ArrayIN[i-kaAhora].f-ArrayIN[i].f;
		b1=ArrayIN[i-kaAhora].c-ArrayIN[i].c;
      
		Curvatura[2*cont]=CurvLocal(f0,f1,b0,b1);
  
		signo=(int)(Curvatura[2*cont]/fabs(Curvatura[2*cont])) ;
		Curvatura[2*cont]=fabs(Curvatura[2*cont]);
		Curvatura[2*cont+1]=(double)signo;
 
		cont++;
      
		// Valor de distkdAhora para i+1
		if (i+kdAhora+1<TamanoLaser)
			{
			distkdAhora=distkdAhora
				-DistEuclidea(ArrayIN[i].f,ArrayIN[i].c,ArrayIN[i+1].f,ArrayIN[i+1].c)
				+DistEuclidea(ArrayIN[i+kdAhora].f,ArrayIN[i+kdAhora].c,
							  ArrayIN[i+kdAhora+1].f,ArrayIN[i+kdAhora+1].c);
	  
			// Valores de kdAhora y distkdAhora actualizados para i+1
			// A. Si DistEuclidea((i+1) a (i+1+kdAhora))+UmbralAdapt < distkdAhora -> reducir kdAhora
			if (DistEuclidea(ArrayIN[i+1].f,ArrayIN[i+1].c,
							 ArrayIN[i+kdAhora+1].f,ArrayIN[i+kdAhora+1].c)+UmbralAdapt<distkdAhora 
				&& kdAhora>KMin)
				{
				distkdAhora=distkdAhora-DistEuclidea(ArrayIN[i+kdAhora].f,ArrayIN[i+kdAhora].c,
													 ArrayIN[i+kdAhora+1].f,ArrayIN[i+kdAhora+1].c);
				kdAhora--;
				}
			// B. Si DistEuclidea+UmbralAdapt > distkdAhora -> aumentar kdAhora (si puede)
			if (DistEuclidea(ArrayIN[i+1].f,ArrayIN[i+1].c,
							 ArrayIN[i+kdAhora+1].f,ArrayIN[i+kdAhora+1].c)+UmbralAdapt>distkdAhora 
				&& kdAhora<KMax && i+kdAhora+2<TamanoLaser)
				{
				kdAhora++;
				distkdAhora=distkdAhora+DistEuclidea(ArrayIN[i+kdAhora].f,ArrayIN[i+kdAhora].c,
													 ArrayIN[i+kdAhora+1].f,ArrayIN[i+kdAhora+1].c);
				}
			}
		else // No quedan puntos en ArrayIN, hay que recortar, forzosamente, kdAhora
			{
			distkdAhora=distkdAhora-DistEuclidea(ArrayIN[i].f,ArrayIN[i].c,
												 ArrayIN[i+1].f,ArrayIN[i+1].c);
			kdAhora--;
			}
      
		// Valor de distkaAhora para i+1
		distkaAhora=distkaAhora
			+DistEuclidea(ArrayIN[i].f,ArrayIN[i].c,ArrayIN[i+1].f,ArrayIN[i+1].c)
			-DistEuclidea(ArrayIN[i-kaAhora].f,ArrayIN[i-kaAhora].c,
						  ArrayIN[i-kaAhora+1].f,ArrayIN[i-kaAhora+1].c);
      
		// Valores de kaAhora y distkaAhora actualizados para i+1
		// A. Si DistEuclidea((i+1) a (i+1-kaAhora))+UmbralAdapt < distkaAhora -> reducir kaAhora
		if (DistEuclidea(ArrayIN[i+1].f,ArrayIN[i+1].c,ArrayIN[i-kaAhora+1].f,
						 ArrayIN[i-kaAhora+1].c)+UmbralAdapt<distkaAhora 
			&& kaAhora>KMin)
			{
			distkaAhora=distkaAhora-DistEuclidea(ArrayIN[i-kaAhora+1].f,ArrayIN[i-kaAhora+1].c,
												 ArrayIN[i-kaAhora+2].f,ArrayIN[i-kaAhora+2].c);
			kaAhora--;
			}
		// B. Si DistEuclidea+UmbralAdapt > distkaAhora -> aumentar kaAhora (siempre puede)
		if (DistEuclidea(ArrayIN[i+1].f,ArrayIN[i+1].c,
						 ArrayIN[i-kaAhora+1].f,ArrayIN[i-kaAhora+1].c)+UmbralAdapt>distkaAhora 
			&& kaAhora<KMax)
			{
			distkaAhora=distkaAhora+DistEuclidea(ArrayIN[i-kaAhora].f,ArrayIN[i-kaAhora].c,
												 ArrayIN[i-kaAhora+1].f,ArrayIN[i-kaAhora+1].c);
			kaAhora++;
			}
		}
  
	TamanoLaser=TamanoLaser-2*KMin-1; /* Actualizamos TamanoLaser */

	//Normalizacion de la curvatura
	for (i=0;i<TamanoLaser;i++)
		{
		CurvDef[2*i]=Curvatura[2*i];
		CurvDef[2*i+1]=Curvatura[2*i+1];
		}

	return(error);
}

int CURVATURALASER::ObtenerCurvaturaKcte()
{
	register int i;
	int error=0;
  
	TamanoLaser=TamanoLaser-K-1;
  
	if (TamanoLaser<=1)
		return(1);	/* error=1 */
  
	for (i=0;i<TamanoLaser+1;i++)
		{
		Curvatura[2*i]=(double)atan2((double)(ArrayIN[i+K].f-ArrayIN[i].f), 
									(double)(ArrayIN[i+K].c-ArrayIN[i].c));
		}
  
	for (i=0;i<TamanoLaser;i++)
		{
		CurvDef[i]=/*Curvatura[i+1]-*/Curvatura[2*i];
		}
  
	return(error);
}

double CURVATURALASER::EsquinaVirtual()
{
	int offset =1;
	return CurvDef[2*offset+1];
}

int CURVATURALASER::DetectarCirculos()
{

	int i; 
	double media, varianza;
	double /*r,xc,yc,*/index_c;

	double max;


	media=0.0;

#ifdef __DEBUG__
	printf("n_puntos: %d\n",TamanoLaser);
#endif

	max=fabs(CurvDef[0]);
	for (i=0;i<TamanoLaser;i++) {
		media+=fabs(CurvDef[2*i]);
		if (fabs(CurvDef[2*i]) >= max) max=fabs(CurvDef[2*i]);
	}

	media=media/TamanoLaser;
	index_c=fabs(media/max); 

	varianza=0;
	for (i=0;i<TamanoLaser;i++) {
		varianza+=(fabs(CurvDef[2*i])-media)*(fabs(CurvDef[2*i])-media);
	}
	
	Circles[0]=TamanoLaser; Circles[1]=media; Circles[2]=varianza; Circles[3]=index_c;

	if ((fabs(Circles[1])>0.020) && (Circles[2]<0.10) && (Circles[3]>0.10) )
	{								
		return 1;
	}
	else return 0;
	



}

int CURVATURALASER::DetectarCirculos(int inicio,int fin)
{
        int i; //int k;
	double media, varianza;
	double /*r,xc,yc,*/index_c;

	double max;
	media=0.0;

#ifdef __DEBUG__
	printf("n_puntos: %d\n", fin - inicio);
	printf("inicio: %d\n",inicio);
	printf("fin: %d\n",fin);
#endif

	max=fabs(CurvDef[0]);
	
	for (i=inicio;i<fin - KMin ;i++) {

#ifdef DEBUG
		printf("c[i]: %f\n", CurvDef[2*i]); 
#endif
		media+=CurvDef[2*i];

		if (fabs(CurvDef[2*i]) >= max) max=fabs(CurvDef[2*i]);
	
	}

	media=media/(fin-inicio-KMin);
	index_c=fabs(media/max); 

	varianza=0;
	for (i=inicio;i<fin-KMin;i++) {
		varianza+=(CurvDef[2*i]-media)*(CurvDef[2*i]-media);
	}
	
	Circles[0]=TamanoLaser; Circles[1]=media; Circles[2]=varianza; Circles[3]=index_c;

	if ((fabs(Circles[1])>0.020) && (Circles[2]<0.10) && (Circles[3]>0.10) )
	{								
		return 1;
	}
	else return 0;

	
}

void CURVATURALASER::DetectarEsquinas()
{
	int i; 
	int ANGLE;
	int SIGNO;
	int _X;
	int _Y;
	int INDICE;
	int REGION; 
	int n;
	int offset;
    int offset_corners = 9;

	ANGLE=5;
	REGION=6;
	_X=2;
	_Y=3;
	INDICE=1;
	SIGNO=4;
	offset=KMin; //2;
	n=0;
  
	Esquinas[0]=-1;

	i=0; 
	if (CurvDef[0]>CurvDef[2] && CurvDef[0]>UmbralEsquina)
		{
		Esquinas[n*offset_corners+_X]=ArrayIN[i+offset].f;
		Esquinas[n*offset_corners+_Y]=ArrayIN[i+offset].c;
		Esquinas[n*offset_corners+INDICE]=i+offset;
	  
		// Caracterizacin de la esquina
		Esquinas[n*offset_corners+SIGNO]=CurvDef[1];
		Esquinas[n*offset_corners+ANGLE]=CurvDef[0]; 

		n++;
		}

	for (i=1;i<TamanoLaser-1;i++)
		{
		if (CurvDef[2*i]>CurvDef[2*(i-1)] && CurvDef[2*i]>CurvDef[2*(i+1)] 
			&& CurvDef[2*i]>UmbralEsquina)
			{
			Esquinas[n*offset_corners+_X]=ArrayIN[i+offset].f;
			Esquinas[n*offset_corners+_Y]=ArrayIN[i+offset].c;
			Esquinas[n*offset_corners+INDICE]=i+offset;

			// Caracterizacin de la esquina
			Esquinas[n*offset_corners+SIGNO]=CurvDef[2*i+1];
			Esquinas[n*offset_corners+ANGLE]=CurvDef[2*i]; 
				
			n++;
			}
		}

	if (CurvDef[2*i]>CurvDef[2*(i-1)] && CurvDef[2*i]>UmbralEsquina)
		{
		Esquinas[n*offset_corners+_X]=ArrayIN[i].f;
		Esquinas[n*offset_corners+_Y]=ArrayIN[i].c;
		Esquinas[n*offset_corners+INDICE]=i;

		// Caracterizacin de la esquina
		Esquinas[n*offset_corners+SIGNO]=CurvDef[2*i+1];
		Esquinas[n*offset_corners+ANGLE]=CurvDef[2*i]; 
		n++;
		}

	Esquinas[0]=n;
}

void CURVATURALASER::LeerArray(char *nombre)
{
	FILE* fd;
	int i;
  
	fd=fopen(nombre, "rb");
	if (!fd) 
		std::cout << "Error abriendo " << nombre << std::endl;
  
	for (i=0;i<TamanoLaser;i++)
		fscanf(fd,"%g %g\n",&ArrayIN[i].f,&ArrayIN[i].c);
	
	fclose(fd);
}

void CURVATURALASER::DataInPixels(double *pixels, int inicio)
{
	register int i,k;
	k=0;
	
	for (i=inicio;i<inicio+TamanoLaser;i++)
		{
		ArrayIN[k].f=pixels[i*3+1]; 
		ArrayIN[k].c=pixels[i*3+2]; 
		k++;
		}
}

void CURVATURALASER::PintarCurvatura(int grupo, int contador)
{
	int i;
	FILE *punt;
	static int cont;
	static int contkd;
	static int contka;
	char name[20];

	sprintf(name,"curv%d.m",contador);
	if (grupo==0) {cont=0;punt=fopen(name,"wb");fprintf(punt,"figure(2);hold on;grid on;zoom on;\n");fclose(punt);}

	punt=fopen(name,"a");
	

	for (i=1;i<TamanoLaser;i++)
		{
		if (grupo==0 || grupo==5)
			fprintf(punt, "plot(%d,%f,'yx')\n",cont++,CurvDef[2*i+1]*CurvDef[2*i]);
		if (grupo==1 || grupo==6)
			fprintf(punt, "plot(%d,%f,'cx')\n",cont++,CurvDef[2*i+1]*CurvDef[2*i]);
		if (grupo==2 || grupo==7)
			fprintf(punt, "plot(%d,%f,'kx')\n",cont++,CurvDef[2*i+1]*CurvDef[2*i]);
		if (grupo==3 || grupo==8)
			fprintf(punt, "plot(%d,%f,'rx')\n",cont++,CurvDef[2*i+1]*CurvDef[2*i]);
		if (grupo==4 || grupo==9)
			fprintf(punt, "plot(%d,%f,'gx')\n",cont++,CurvDef[2*i+1]*CurvDef[2*i]);
		}
	fclose(punt);

	if (grupo==0) {contkd=0;punt=fopen("kd.m","wb");fclose(punt);}

	punt=fopen("kd.m","a");
	fprintf(punt,"hold on\n");

	for (i=KMin;i<TamanoLaser-KMin;i++)
		{
		if (grupo==0 || grupo==5)
			fprintf(punt, "plot(%d,%d,'.b')\n",contkd++,ArrayIN[i].kd);
		if (grupo==1 || grupo==6)
			fprintf(punt, "plot(%d,%d,'.c')\n",contkd++,ArrayIN[i].kd);
		if (grupo==2 || grupo==7)
			fprintf(punt, "plot(%d,%d,'.k')\n",contkd++,ArrayIN[i].kd);
		if (grupo==3 || grupo==8)
			fprintf(punt, "plot(%d,%d,'.r')\n",contkd++,ArrayIN[i].kd);
		if (grupo==4 || grupo==9)
			fprintf(punt, "plot(%d,%d,'.g')\n",contkd++,ArrayIN[i].kd);
		}
	fclose(punt);

	if (grupo==0) {contka=0;punt=fopen("ka.m","wb");fclose(punt);}

	punt=fopen("ka.m","a");
	fprintf(punt,"hold on\n");

	for (i=KMin;i<TamanoLaser-KMin;i++)
		{
		if (grupo==0 || grupo==5)
			fprintf(punt, "plot(%d,%d,'.b')\n",contka++,ArrayIN[i].ka);
		if (grupo==1 || grupo==6)
			fprintf(punt, "plot(%d,%d,'.c')\n",contka++,ArrayIN[i].ka);
		if (grupo==2 || grupo==7)
			fprintf(punt, "plot(%d,%d,'.k')\n",contka++,ArrayIN[i].ka);
		if (grupo==3 || grupo==8)
			fprintf(punt, "plot(%d,%d,'.r')\n",contka++,ArrayIN[i].ka);
		if (grupo==4 || grupo==9)
			fprintf(punt, "plot(%d,%d,'.g')\n",contka++,ArrayIN[i].ka);
		}
	fclose(punt);

}

// Comment: "Actualizacion"

double *CURVATURALASER::PintarCurvaturaScan(int id, int *size) 
{ // OJO: no se usa esta funcion pero al utilizarla hay que liberar memoria // (added by Ricardo Vazquez)
  int i;
  FILE *punt;
  double *result;
  
  char nombre[20];
  sprintf(nombre,"curvScan%d.m",id);
  punt=fopen(nombre,"wb");
  fprintf(punt,"figure(2);hold on; zoom on; grid on; xlabel('Range reading');ylabel('Curvature');\n");
  fprintf(punt,"y=%f;",CurvDef[0]);

  result=(double*)malloc((TamanoLaser)*sizeof(double));

  for (i=1;i<TamanoLaser;i++) {
    fprintf(punt,"y=[y %f];",CurvDef[2*i+1]*CurvDef[2*i]);
    result[i-1]=(CurvDef[2*i+1]*CurvDef[2*i]);
  }
  
  fprintf(punt,"x=[0:1:%d-1];",TamanoLaser);
  
  if (id==1) 
    fprintf(punt,"plot(x,y,'r-')");
  else 
    fprintf(punt,"plot(x,y,'-')");

  fclose(punt);
  *size = TamanoLaser;

  return result;
}

int CURVATURALASER::ScanMatchingCurvature(double *fc,int size_fc, int *size_fc_local)
{

  int i;

  *size_fc_local=TamanoLaser;

  for (i=0;i<TamanoLaser;i++) {
    fc[size_fc+i]=CurvDef[2*i+1]*CurvDef[2*i];
  }

  return size_fc+TamanoLaser;

}

