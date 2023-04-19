

#ifndef PROJECTMAIN_H_
#define PROJECTMAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif
    void ProjectMain();

    void i2c_transfer_complete_callback(void);
    void i2c_transfer_error_callback(void);
    void i2c_receive_complete_callback(void);
    void i2c_receive_error_callback(void);
    void Error_Callback(void);
    
#ifdef __cplusplus
}
#endif

#endif /* PROJECTMAIN_H_ */
