/*------------------------------------------------------------------------------------------------*/
/*	KibaControl		Kennlinien und -interpolation	                                */
/*					PID-Regler mit Gain-Scheduling und Anti-Windup																				*/
/*																																																*/
/*	Frank Kirschbaum (frank.kirschbaum@me.com)                                 					     */
/*																								     */
/*																								     */
/*	Kibas Coding Standard and Style Guide:                                    					     */
/*  (frei nach http://www.freertos.org/FreeRTOS-Coding-Standard-and-Style-Guide.html)			     */
/*																								     */
/*	Namenskonventionen:																			     */
/*	Präfixes für Konstanten, Variablen, Funktionen und Methoden:								     */
/*	void/void*                   v/pv    void                                                        */
/*	int/int*                     i/pi    integer                                                     */
/*	uint/uint*                   ui/pui  unsigned integer                                            */
/*	int8_t/int8_t*               c/pc    char (Byte)                                                 */
/*	uint8_t/uint8_t*             uc/puc  unsigned char										         */
/*	int16_t/int16_t*             s/ps    short                                                       */
/*	uint16_t/uint16_t*           us/pus  unsigned short                                              */
/*	int32_t/int32_t*             l/pl    long                                                        */
/*	uint32_t/uint32_t*           ul/pul  unsigned long                                               */
/*	char/unsigned char           uc/puc  char (byte) für Zeichen                                     */
/*	float/float*                 f/pf    float                                                       */
/*	double/double*               d/pd    double                                                      */
/*	BaseType_t/BaseType_t*       x/px    base type, optimal für Registerbreite                       */
/*	UBaseType_t/UBaseType_t*     ux/pux  unsigned base type, optimal für Registerbreite              */
/*	TickType_t/TickType_t*       x/px    16/32 Bit, abhängig von Registerbreite                      */
/*	size_t/size_t*               x/px                                                                */
/*	TaskHandle_t                 pv      Task-handle (Pointer) für die Referenzierung von Tasks      */
/*	SemaphoreHandle_t            x                                                                   */
/*	Postfix:                                                                                         */
/*	class member variables       XYZ_		Unterstrich am Ende jeder Member-Variablen               */
/*                                                                                                   */
/*	Lesbarkeit des Quelltextes:	                                                                     */
/*	Space nach ( und vor )                                                                           */
/*	Space nach [ und vor ]                                                                           */
/*	unter jeder Funktionsdeklaration etc. ----...---                                                 */
/*                                                                                                   */
/*	'Der Unterschied zwischen Theorie und Praxis ist in der Praxis größer als in der Theorie'        */
/*                                                                                                   */
/*---------------------------------------------------------------------------------------------------*/
#include <KibaControl.h>
#include <stdint.h>
/*------------------------------------------------------------------------------------------------*/
/*
 * liefert größstes Element eines Arrays
 */
 int iMaxElement( int* piArray, int8_t ucSize )
 {
	 int iDummy = INT_MIN; // iDummy = kleinstmögliche Zahl
	 for (int8_t uc=0; uc < ucSize; uc++) // Schleife über alle Stützstellen
	 {
		 if ( piArray[ uc ] > iDummy ) // wenn akluelles Element größer
		 {
			 iDummy = piArray[ uc ]; // iDummy = neues größtes Element
		 }
	 }
	 return iDummy;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * liefert kleinstes Element eines Arrays
 */
 int iMinElement( int* piArray, int8_t ucSize )
 {
	 int iDummy = INT_MAX;     // iDummy = größtmögliche Zahl
	 for (int8_t uc=0; uc < ucSize; uc++) // Schleife über alle Stützstellen
	 {
		 if ( piArray[ uc ] < iDummy ) // wenn akluelles Element kleiner
		 {
			 iDummy = piArray[ uc ]; // iDummy = neues kleinstes Element
		 }
	 }
	 return iDummy;
 }
 /*------------------------------------------------------------------------------------------------*/
 /*
  * Kennlinieninterpolation
 */
 int map1D::iOut( int iIn )
 {
	 if ( iIn <= piInVec_[ 0 ] ) return piOutVec_[ 0 ]; // unterhalb des Eingangsbereichs -> konst. Extrap.
	 if ( iIn >= piInVec_[ucVecSize_-1] ) return piOutVec_[ ucVecSize_-1 ]; // oberhalb des Eingangsbereichs -> konst. Extrap.

	 uint8_t ucPos = 1;         // piInVec[0] bereits überprüft
	 while( iIn > piInVec_[ ucPos ] ) ucPos++;  // Intervall suchen

	 if ( iIn == piInVec_[ ucPos ] ) return piOutVec_[ ucPos ]; // Behandlung 'exakter' Punkte

	 // Interpolation zwischen den Stützstellen
	 return ( iIn - piInVec_[ ucPos-1 ] ) * ( piOutVec_[ ucPos ] - piOutVec_[ ucPos-1 ] ) / ( piInVec_[ ucPos ] - piInVec_[ ucPos-1 ] ) + piOutVec_[ ucPos-1 ];
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzen bzw. Definieren einer Kennlinie
 */
 uint8_t map1D::ucSetMap( int* piIn, int* piOut, uint8_t ucSize )
 {
	 int iInSize = piIn[ 0 ];    // bisher kleinstes Element = erstes Element
	 for (uint8_t uc=1; uc <= ucSize-1; uc++ ) // Schleife über alle Stützstellen
	 {
		 if ( (piIn[ uc ] < iInSize ) ) // kleiner als bisher kleinstes?
		 {
			 return 1; // Fehler! keine Monotonie!
		 }
		 iInSize = piIn[ uc ]; // bisher kleinstes Element = aktuelles Element
	 }
	 piInVec_ = piIn;      // Übergabe Zeiger auf Stützstellenvektor
	 piOutVec_ = piOut;      // Übergabe Zeiger auf Vektor mit den Werten an den Stützstellen
	 ucVecSize_ = ucSize;     // Übergabe Anzahl der Stützstellen

	 iInMin_ = piInVec_[ 0 ];     // kleinste Stützstelle
	 iInMax_ = piInVec_[ ucVecSize_-1 ];  // größte Stützstelle

	 iOutMax_ = iMaxElement( piOutVec_, ucVecSize_ ); // maximale Ausgangsgröße
	 iOutMin_ = iMinElement( piOutVec_, ucVecSize_ ); // minimale Ausgangsgröße

	 return 0;         // kein fehler
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Liefert kleinsten möglichen Ausgangswert einer Kennlinie
 */
int map1D::iGetMinOut( void )
{
		return iOutMin_;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Liefert größten möglichen Ausgangswert einer Kennlinie
 */
 int map1D::iGetMaxOut( void )
 {
	 return iOutMax_;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Liefert untere Grenze des Wertebereichs der x-Achse
 */
 int map1D::iGetMinSupportingPoint( void )
 {
	 return iInMin_;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Liefert obere Grenze des Wertebereichs der x-Achse
 */
 int map1D::iGetMaxSupportingPoint( void )
 {
	 return iInMax_;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Konstruktor eines Objekts der Klasse 'PIDcontrol'
 */
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
/*------------------------------------------------------------------------------------------------*/
/*
 * Berechnet den Ausgang eines PID-Reglers mit Gain-Scheduling und Anti-Windup
 */
 int PIDcontrol::iOut( int iDesiredValue, int iActualValue )
 {
	 int iError;
	 int iSum;

	 iError = iDesiredValue - iActualValue;

	 if (!ucIsLimited_)
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
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzen der Proportionalverstärkung
 */
 void PIDcontrol::vSetProportionalGain(int iKP)
 {
	 iProportionalGain_ = iKP;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzen der Integralverstärkung
 */
 void PIDcontrol::vSetIntegrationalGain(int iKI)
 {
	 iIntegrationalGain_ = iKI;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt die Differentialverstärkung
 */
 void PIDcontrol::vSetDifferentialGain(int iKD)
 {
	 iDifferentialGain_ = iKD;
 }
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt die untere Reglerausgangsbeschränkung
 */
void PIDcontrol::vSetOutMinLimit( int iOutMin )
{
	iOutMinLim_ = iOutMin;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt die obere Reglerausgangsbeschränkung
 */
void PIDcontrol::vSetOutMaxLimit( int iOutMax )
{
	iOutMaxLim_ = iOutMax;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt das Gain-Scheduling für den Proportionalanteil
 */
void PIDcontrol::vSetProportionalGainScheduling( map1D Pmap )
{
	ProportionalMap_ = Pmap;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt das Gain-Scheduling für den Integralanteil
 */
void PIDcontrol::vSetIntegrationalGainScheduling( map1D Imap )
{
	IntegrationalMap_ = Imap;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt das Gain-Scheduling für den Differentialanteil
 */
void PIDcontrol::vSetDifferentialGainScheduling( map1D Dmap )
{
	DifferentialMap_ = Dmap;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Setzt den Integrierer zurück auf '0'
 */
void PIDcontrol::vResetIntegrator( void )
{
	iIntegrator_ = 0;
	ucIsLimited_ = 0;
}
/*------------------------------------------------------------------------------------------------*/
/*
 * Liefert den Anti-Windup-Status (1 = Reglerausgangsbeschränkung aktiv, 0 = Reglerausgangsbeschränkung nicht aktiv)
 */
uint8_t PIDcontrol::ucGetAntiWindupState( void )
{
	return ucIsLimited_;
}
/*------------------------------------------------------------------------------------------------*/
