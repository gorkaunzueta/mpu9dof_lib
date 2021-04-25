project(CFE_MPU9DOF_LIB C)

# Create the app module
add_cfe_app(mpu9dof_lib fsw/src/mpu9dof_lib.c)

# Add dependency to the bcm2835 to have access to the i2c functions
add_cfe_app_dependency(mpu9dof_lib bcm2835_lib)

# The API to this library (which may be invoked/referenced from other apps)
# is stored in fsw/public_inc.  Using "target_include_directories" is the 
# preferred method of indicating this (vs. directory-scope "include_directories").
target_include_directories(bcm2835_lib PUBLIC fsw/public_inc)



