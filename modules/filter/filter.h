#ifndef LIBRARIES_TERALIB_MODULES_MOVING_AVERAGE_FILTER_MOVING_AVERAGE_FILTER_H_INCLUDED
#define LIBRARIES_TERALIB_MODULES_MOVING_AVERAGE_FILTER_MOVING_AVERAGE_FILTER_H_INCLUDED


/**
 * A running moving average (RMA) filter is essentially a exponential moving
 * average (EMA) with alpha=1/N
 * @param old_RMA       The previous result of the RMA filter
 * @param window_size   The length of the filter
 * @param new_value     The next value put into the filter
 * @result The new RMA
 */
int32_t TERA_FILTER_RunningMovingAverage(int32_t old_RMA, uint32_t window_size, int32_t new_value){
  uint32_t N = window_size;
  int32_t result = ((N-1) * old_RMA + new_value) / N;
  return result;
}

/**
 * This is the simplest form of a moving average filter. It just averages
 * the data values.
 * @param old_CMA       The previous result of the CMA filter
 * @param window_size   The length of the filter
 * @param new_value     The next value put into the filter
 * @result The new CMA
 */
int32_t TERA_FILTER_CummulativeMovingAverage(int32_t old_CMA, uint32_t window_size, int32_t new_value){
  uint32_t N = window_size;
  int32_t result = old_CMA + (new_value - old_CMA) / (N+1);
  return result;
}



#endif /* LIBRARIES_TERALIB_MODULES_MOVING_AVERAGE_FILTER_MOVING_AVERAGE_FILTER_H_INCLUDED */
