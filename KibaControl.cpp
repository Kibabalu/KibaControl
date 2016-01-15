#include "KibaControl.h"
#include <stdint.h>
/*--------------------------------------------------------------------------------------------------*/


int iMaxElement( int* piArray, int8_t ucSize )
{
	int iDummy = INT_MIN;												// iDummy = kleinstmögliche Zahl
	for (int8_t uc=0; uc < ucSize; uc++)								// Schleife über alle Stützstellen
	{
		if ( piArray[ uc ] > iDummy )									// wenn akluelles Element größer
		{
			iDummy = piArray[ uc ];										// iDummy = neues größtes Element
		}
	}
	return iDummy;
}
/*--------------------------------------------------------------------------------------------------*/

int iMinElement( int* piArray, int8_t ucSize )
{
	int iDummy = INT_MAX;												// iDummy = größtmögliche Zahl
	for (int8_t uc=0; uc < ucSize; uc++)								// Schleife über alle Stützstellen
	{
		if ( piArray[ uc ] < iDummy )									// wenn akluelles Element kleiner
		{
			iDummy = piArray[ uc ];										// iDummy = neues kleinstes Element
		}
	}
	return iDummy;
}
/*--------------------------------------------------------------------------------------------------*/

int map1D::iOut( int iIn )
{
	if ( iIn <= piInVec_[ 0 ] ) return piOutVec_[ 0 ];					// unterhalb des Eingangsbereichs -> konst. Extrap.
  	if ( iIn >= piInVec_[ucVecSize_-1] ) return piOutVec_[ ucVecSize_-1 ];	// oberhalb des Eingangsbereichs -> konst. Extrap.

  	uint8_t ucPos = 1;  												// piInVec[0] bereits überprüft
  	while( iIn > piInVec_[ ucPos ] ) ucPos++;							// Intervall suchen

  	if ( iIn == piInVec_[ ucPos ] ) return piOutVec_[ ucPos ];			// Behandlung 'exakter' Punkte
  	
  	// Interpolation zwischen den Stützstellen
  	return ( iIn - piInVec_[ ucPos-1 ] ) * ( piOutVec_[ ucPos ] - piOutVec_[ ucPos-1 ] ) / ( piInVec_[ ucPos ] - piInVec_[ ucPos-1 ] ) + piOutVec_[ ucPos-1 ];
}
/*--------------------------------------------------------------------------------------------------*/

uint8_t map1D::ucSetMap( int* piIn, int* piOut, uint8_t ucSize )
{
	int iInSize = piIn[ 0 ];											// bisher kleinstes Element = erstes Element
	for (uint8_t uc=1; uc <= ucSize-1; uc++ )							// Schleife über alle Stützstellen
	{
		if ( (piIn[ uc ] < iInSize ) )									// kleiner als bisher kleinstes?
		{
			return 1;													// Fehler! keine Monotonie! 		
		}
		iInSize = piIn[ uc ];											// bisher kleinstes Element = aktuelles Element
	}
	piInVec_ = piIn;													// Übergabe Zeiger auf Stützstellenvektor
	piOutVec_ = piOut;													// Übergabe Zeiger auf Vektor mit den Werten an den Stützstellen
	ucVecSize_ = ucSize;												// Übergabe Anzahl der Stützstellen
	
	iInMin_ = piInVec_[ 0 ]; 											// kleinste Stützstelle
	iInMax_ = piInVec_[ ucVecSize_-1 ];									// größte Stützstelle
	
	iOutMax_ = iMaxElement( piOutVec_, ucVecSize_ );					// maximale Ausgangsgröße
	iOutMin_ = iMinElement( piOutVec_, ucVecSize_ );					// minimale Ausgangsgröße
	
	return 0;														 	// kein fehler
}
/*--------------------------------------------------------------------------------------------------*/

int map1D::iGetMinOut( void )
{
	return iOutMin_;
}
/*--------------------------------------------------------------------------------------------------*/

int map1D::iGetMaxOut( void )
{
	return iOutMax_;
}
/*--------------------------------------------------------------------------------------------------*/

int map1D::iGetMinSupportingPoint( void )
{
	return iInMin_;
}
/*--------------------------------------------------------------------------------------------------*/

int map1D::iGetMaxSupportingPoint( void )
{
	return iInMax_;
}
/*--------------------------------------------------------------------------------------------------*/

PIDcontrol::PIDcontrol( int iKP, int iKI, int iKD, int iOutMin, int iOutMax, map1D Pmap, map1D IMap, map1D DMap)
{
	iProportionalGain_ = iKP;
	iIntegrationalGain_ = iKI;
	iDifferentialGain_ = iKD;
	iOutMinLim_ = iOutMin;
	iOutMaxLim_ = iOutMax;
	ProportionalMap_ = Pmap;
	IntegrationalMap_ = IMap;
	DifferentialMap_ = DMap;
}
/*--------------------------------------------------------------------------------------------------*/

int PIDcontrol::iOut( int iDesiredValue, int iActualValue )
{
	int iError;
	int iSum;
	
	iError = iDesiredValue - iActualValue;
	
	if (! ucIsLimited_)
	{
		iIntegrator_ = iIntegrator_ + iError * IntegrationalMap_.iOut( iError );
	}
		
	iSum = iProportionalGain_ * iError + iIntegrator_;
	
	if ( iSum <= iOutMinLim_ )
	{
		ucIsLimited_ = 1;
		return iOutMinLim_;
	}
	else if (iSum >= iOutMaxLim_)
	{
		ucIsLimited_ = 1;
		return iOutMaxLim_;
	}
	else
	{
		ucIsLimited_ = 0;
		return iSum;
	}
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetProportionalGain(int iKP)
{
	iProportionalGain_ = iKP;
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetIntegrationalGain(int iKI)
{
	iIntegrationalGain_ = iKI;
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetDifferentialGain(int iKD)
{
	iDifferentialGain_ = iKD;
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetOutMinLimit( int iOutMin )
{
	iOutMinLim_ = iOutMin;
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetOutMaxLimit( int iOutMax )
{
	iOutMaxLim_ = iOutMax;
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetProportionalGainScheduling( map1D Pmap )
{
	ProportionalMap_ = Pmap;	
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetIntegrationalGainScheduling( map1D Imap )
{
	IntegrationalMap_ = Imap;	
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vSetDifferentialGainScheduling( map1D Dmap )
{
	DifferentialMap_ = Dmap;	
}
/*--------------------------------------------------------------------------------------------------*/

void PIDcontrol::vResetIntegrator( void )
{
	iIntegrator_ = 0;
	ucIsLimited_ = 0;
}
/*--------------------------------------------------------------------------------------------------*/

uint8_t PIDcontrol::ucGetAntiWindupState( void )
{
	return ucIsLimited_;
}
/*--------------------------------------------------------------------------------------------------*/