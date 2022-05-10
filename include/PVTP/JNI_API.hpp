/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class PVTPlanner_JNI_API */

#ifndef _Included_PVTPlanner_JNI_API
#define _Included_PVTPlanner_JNI_API
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetNaiveBottleneckPT_Obstacles
 * Signature: (DDDDDDDDDDDDDD[[D)[[D
 */
JNIEXPORT jobjectArray JNICALL Java_PVTPlanner_JNI_1API_GetNaiveBottleneckPT_1Obstacles
  (JNIEnv *, jclass, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jobjectArray);

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetBottleneckPT_Obstacles
 * Signature: (DDDDDDDDDDD[[D)[[D
 */
JNIEXPORT jobjectArray JNICALL Java_PVTPlanner_JNI_1API_GetBottleneckPT_1Obstacles
  (JNIEnv *, jclass, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jobjectArray);

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    TestScenario
 * Signature: (DDDDDDDDDDD[[D)I
 */
JNIEXPORT jint JNICALL Java_PVTPlanner_JNI_1API_TestScenario
  (JNIEnv *, jclass, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jobjectArray);

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetOptimalTrajectory
 * Signature: (DDDDDDDDDDD[[D)[[D
 */
JNIEXPORT jobjectArray JNICALL Java_PVTPlanner_JNI_1API_GetOptimalTrajectory
  (JNIEnv *, jclass, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jobjectArray);

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetOptimalControl
 * Signature: (DDDDDDDDDDDD[[D)D
 */
JNIEXPORT jdouble JNICALL Java_PVTPlanner_JNI_1API_GetOptimalControl
  (JNIEnv *, jclass, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jdouble, jobjectArray);

#ifdef __cplusplus
}
#endif
#endif