#ifndef HOPPER_MACROS_H_
#define HOPPER_MACROS_H_

#define LINEAR_ACTUATOR_RETRACTED   (0U)
#define LINEAR_ACTUATOR_RETRACTING  (1U)
#define LINEAR_ACTUATOR_CONTRACTED  (2U)
#define LINEAR_ACTUATOR_CONTRACTING (3U)
#define LINEAR_ACTUATOR_UNKNOWN     (4U)

const char* linear_actuator_state_to_str(unsigned linear_actuator_state)  {            
  switch (linear_actuator_state) {
    case LINEAR_ACTUATOR_RETRACTED:   return "LINEAR_ACTUATOR_RETRACTED";   
    case LINEAR_ACTUATOR_RETRACTING:  return "LINEAR_ACTUATOR_RETRACTING";  
    case LINEAR_ACTUATOR_CONTRACTED:  return "LINEAR_ACTUATOR_CONTRACTED";  
    case LINEAR_ACTUATOR_CONTRACTING: return "LINEAR_ACTUATOR_CONTRACTING"; 
    default:                          return "LINEAR_ACTUATOR_UNKNOWN";     
  }                                                                         
}

#endif  // HOPPER_MACROS_H_