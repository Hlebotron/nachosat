/*!
  @brief Calculates the speed and the position where the sat is supposed to be using the GPS coordinates, then returns the orientation the parachutes have to be facing.
  @param sp	GPS coordinates of the landing spot (Set Point)
  @param pv	Current coordinates (Process Variable)
  @param orient	Current orientation
 */
Orient pid( GPSData sp, GPSData pv, Orient orient );

void ParachuteTask( void* params );
