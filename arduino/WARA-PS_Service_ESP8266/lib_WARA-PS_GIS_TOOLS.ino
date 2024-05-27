/*
 * Simple GIS tools for calculating bearings, headings, distances etc. between geopositions.
 */

float getBearingBetween(GeoPoint gp1, GeoPoint gp2){
  return getBearingBetween(gp1.lat, gp1.lon, gp2.lat, gp2.lon);
}

float getBearingBetween(float lat1, float lon1, float lat2, float lon2){
  float lat1r = d2r(lat1);
  float lon1r = d2r(lon1);
  float lat2r = d2r(lat2);
  float lon2r = d2r(lon2);

  //determine angle
  float bearing_r = atan2(sin(lon2r-lon1r)*cos(lat2r), (cos(lat1r)*sin(lat2r))-(sin(lat1r)*cos(lat2r)*cos(lon2r-lon1r)));
  
  //convert to degrees
  float bearing_d = r2d(bearing_r);

  //use mod to turn -90 = 270
  bearing_d = fmod((bearing_d + 360.0), 360);
  
  return bearing_d;
}

float getDistanceBetween(GeoPoint gp1, GeoPoint gp2) {
  return getDistanceBetween(gp1.lat, gp1.lon, gp2.lat, gp2.lon);
}

float getDistanceBetween(float lat1, float lon1, float lat2, float lon2){
  int   R         = 6371000;        // Radius of the earth in meters
  float lat_diff  = d2r(lat2-lat1);
  float lon_diff  = d2r(lon2-lon1); 
  float a         = sin(lat_diff/2) * (lat_diff/2) + cos(d2r(lat1)) * cos(d2r(lat2)) * sin(lon_diff/2) * sin(lon_diff/2); 
  float c         = 2 * atan2(sqrt(a), sqrt(1-a)); 
  float d         = R * c;          // Distance in meters
  return d;
}

float d2r(float deg) {
  return deg * (PI/180);
}

float r2d(float rad) {
  return rad * (180/PI);
}
