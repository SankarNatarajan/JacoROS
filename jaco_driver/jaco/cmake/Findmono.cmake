find_package(PkgConfig)

FIND_PATH( mono-2.0_INCLUDE_PATH mono/jit/jit.h
		/usr/include/mono-1.0
		/usr/include/mono-2.0
		/usr/local/include
		/opt/local/include
		DOC "The directory where mono/jit/jit.h resides")
FIND_LIBRARY( mono-2.0_LIBRARY
		NAMES mono-2.0
		PATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/opt/local/lib
		DOC "The mono-2.0 library")

IF (mono-2.0_INCLUDE_PATH AND mono-2.0_LIBRARY)
	SET( mono-2.0_FOUND 1 CACHE STRING "Set to 1 if mono-2.0 is found, 0 otherwise")
	SET( mono-2.0_INCLUDE_DIRS ${mono-2.0_INCLUDE_PATH})
	SET( mono-2.0_INCLUDE_LIBS ${mono-2.0_LIBRARY})
ELSE(mono-2.0_INCLUDE_PATH AND mono-2.0_LIBRARY)
	SET( mono-2.0_FOUND 0 CACHE STRING "Set to 1 if mono-2.0 is found, 0 otherwise")
ENDIF(mono-2.0_INCLUDE_PATH AND mono-2.0_LIBRARY)

MARK_AS_ADVANCED( mono-2.0_FOUND mono-2.0_INCLUDE_LIBS mono-2.0_INCLUDE_DIRS)
