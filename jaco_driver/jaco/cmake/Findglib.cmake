find_package(PkgConfig)

FIND_PATH( glib-2.0_INCLUDE_PATH glib.h 
		/usr/include/glib-2.0/
		/usr/local/include
		/opt/local/include
		/usr/lib/x86_64-linux-gnu/glib-2.0/include
		DOC "The directory where glib-2.0/glib.h resides")

FIND_PATH( glib-2.0_CONFIG_INCLUDE_PATH glibconfig.h
		/usr/lib/x86_64-linux-gnu/glib-2.0/include
		/usr/lib/glib-2.0/include
		DOC "The directory where glib-2.0/glibconfig.h resides")

FIND_LIBRARY( glib-2.0_LIBRARY
		NAMES glib-2.0
		PATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/opt/local/lib
		DOC "The glib-2.0 library")

IF (glib-2.0_INCLUDE_PATH)
	SET( glib-2.0_FOUND 1 CACHE STRING "Set to 1 if glib-2.0 is found, 0 otherwise")
	SET( glib-2.0_INCLUDE_DIRS ${glib-2.0_INCLUDE_PATH} ${glib-2.0_CONFIG_INCLUDE_PATH})
	SET( glib-2.0_INCLUDE_LIBS ${glib-2.0_LIBRARY})
ELSE (glib-2.0_INCLUDE_PATH)
	SET( glib-2.0_FOUND 0 CACHE STRING "Set to 1 if glib-2.0 is found, 0 otherwise")
ENDIF (glib-2.0_INCLUDE_PATH)

MARK_AS_ADVANCED( glib-2.0_FOUND glib-2.0_INCLUDE_LIBS glib-2.0_INCLUDE_DIRS)
