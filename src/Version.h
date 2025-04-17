/*
 * Version.h
 *
 *  Created on: 1 Sep 2019
 *      Author: David
 */

#ifndef SRC_VERSION_H_
#define SRC_VERSION_H_

#define VERSION		"3.6.0-rc.2+4"

#if 0
// Use this for official releases
# define TIME_SUFFIX
#else
// Use this for internal builds
# define TIME_SUFFIX		" " __TIME__
#endif

extern const char *const DateText;

#endif /* SRC_VERSION_H_ */
