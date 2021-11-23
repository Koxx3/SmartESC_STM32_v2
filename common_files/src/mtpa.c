__weak void MTPA_CalcCurrRefFromIq( MTPA_Handle_t * pHandle, qd_t *Iqdref )
{
  if (Iqdref->q < 0){
	  Iqdref->q = -Iqdref->q;
  }

  unsigned int v3 = (uint8_t)((int16_t)Iqdref->q / pHandle->SegDiv);
  if (v3 >= 7){
	  v3 = 7;
  }

  Iqdref->d = (uint16_t)pHandle->Offset[v3] + pHandle->AngCoeff[v3] * Iqdref->q / 0x8000;
}
