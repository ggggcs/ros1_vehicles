#include <math.h>

static double latLongToDegree(const double dddmm_mmmm)
{
  // eg.: 12031.0902 -> 120.51879(120+(31.0902/60.0=0.51917))
  // refer to https://www.ridgesolutions.ie/index.php/2022/05/18/code-example-convert-decimal-degrees-minutes-ddm-to-decimal-degrees-dd/
  double degrees = floor(dddmm_mmmm/100.0);
  double minutes = dddmm_mmmm - (degrees * 100.0);
  double decimal_degreses = degrees + (minutes / 60.0);
  return decimal_degreses;
}

void ddmmtoddd(double lng, double lat, double &out_lng, double &out_lat)
{
  out_lng = latLongToDegree(lng);
  out_lat = latLongToDegree(lat);
}
