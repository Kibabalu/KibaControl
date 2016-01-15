#ifndef KibaControl_h
#define KibaControl_h 1

#include <stdint.h>
#include <limits.h>
/*--------------------------------------------------------------------------------------------------*/

class map1D
{
	private:
		int* piInVec_;													// Stützstellen
		int* piOutVec_;													// Werte an den Stützstellen
		uint8_t ucVecSize_;												// Länge Ein-und Ausgangsvektor
		int iInMin_;													// kleinste Stützstelle
		int iInMax_;													// größte Stützstelle
		int iOutMin_;													// minimale Ausangsgröße
		int iOutMax_;													// maximale Ausgangsgröße																	
	public:
		int iOut( int iIn );											// Kennlinienauswertung
		uint8_t ucSetMap( int* piIn, int* piOut, uint8_t ucSize );		// Kennlinie initialisieren
		int iGetMinOut( void );											// liefert minimal möglichen Ausgangswert
		int iGetMaxOut( void );											// liefert maximal möglichen Ausgangswert
		int iGetMinSupportingPoint( void );								// liefert kleinste Stützstelle
		int iGetMaxSupportingPoint( void );								// liefert größte Stützstelle
};
/*--------------------------------------------------------------------------------------------------*/

class PIDcontrol
{
	private:
	 	int iIntegrator_;												// Integrierer
		int iErrorLastStep_;											// Regelabweichung im letzten Abstastschritt
		int iOutMinLim_;												// untere Reglerausgangsbeschränkung
		int iOutMaxLim_;												// obere Reglerausgangsbeschränkung
		map1D ProportionalMap_; 										// Kennlinie Gain-Scheduling Proportionalanteil
		map1D IntegrationalMap_; 										// Kennlinie Gain-Scheduling Integralanteil
		map1D DifferentialMap_; 										// Kennlinie Gain-Scheduling Differentialanteil
		uint8_t ucIsLimited_;											// Status Ant-Windup
		int iProportionalGain_;											// Kp Verstärkung Proportionalanteil
		int iIntegrationalGain_;										// Ki Verstärkung Integralanteil
		int iDifferentialGain_;											// Kd Verstärkung Differentialanteil
	public:
		PIDcontrol( int iKP, int iKI, int iKD, int iOutMin, int iOutMax, map1D Pmap, map1D IMap, map1D DMap );
		int iOut( int iDesiredValue, int iActualValue );				// Berechnung Reglerausgang
		void vSetProportionalGain( int iKP );							// setze Kp Verstärkung Proportionalanteil 
		void vSetIntegrationalGain( int iKI );							// setze Ki Verstärkung Integralanteil
		void vSetDifferentialGain( int iKD );							// setze Kd Verstärkung Differentialanteil
		void vSetOutMinLimit( int iOutMin );							// setze untere Reglerausgangsbeschränkung
		void vSetOutMaxLimit( int iOutMax );							// setze obere Reglerausgangsbeschränkung
		void vSetProportionalGainScheduling( map1D Pmap );				// setze Kennlinie Gain-Scheduling Proportionalanteil
		void vSetIntegrationalGainScheduling( map1D Imap );				// setze Kennlinie Gain-Scheduling Integralanteil
		void vSetDifferentialGainScheduling( map1D Dmap );				// setze Kennlinie Gain-Scheduling Differentialanteil
		void vResetIntegrator( void ); 									// Reset Integrierer
		uint8_t ucGetAntiWindupState( void );							// Liefert Status Anti-Windup
};

#endif // KibaControl_h