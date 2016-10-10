include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

file(
	GLOB
	Bullet_INCLUDE_PATHS
	$ENV{BULLETDIR}/include/bullet
	$ENV{HOME}/include/bullet
	/usr/local/include/bullet
	/opt/local/include/bullet
	/usr/include/bullet
	$ENV{SystemDrive}/bullet*/include/bullet
	$ENV{ProgramW6432}/bullet*/include/bullet
	$ENV{ProgramFiles}/bullet*/include/bullet
	$ENV{ProgramW6432}/rl*/include/bullet
	$ENV{ProgramFiles}/rl*/include/bullet
)

find_path(
	Bullet_INCLUDE_DIRS
	NAMES
	btBulletCollisionCommon.h
	HINTS
	${Bullet_INCLUDE_PATHS}
)

mark_as_advanced(Bullet_INCLUDE_DIRS)

file(
	GLOB
	Bullet_LIBRARY_PATHS
	$ENV{BULLETDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/opt/local/lib
	/usr/lib
	$ENV{SystemDrive}/bullet*/lib
	$ENV{ProgramW6432}/bullet*/lib
	$ENV{ProgramFiles}/bullet*/lib
	$ENV{ProgramW6432}/rl*/lib
	$ENV{ProgramFiles}/rl*/lib
)

find_library(
	Bullet_BULLETCOLLISION_LIBRARY_DEBUG
	NAMES
	bulletcollisiond BulletCollisiond BulletCollision_debug BulletCollision_Debug
	HINTS
	${Bullet_LIBRARY_PATHS}
)

find_library(
	Bullet_BULLETCOLLISION_LIBRARY_RELEASE
	NAMES
	bulletcollision BulletCollision
	HINTS
	${Bullet_LIBRARY_PATHS}
)

select_library_configurations(Bullet_BULLETCOLLISION)

find_library(
	Bullet_BULLETDYNAMICS_LIBRARY_DEBUG
	NAMES
	bulletdynamicsd BulletDynamicsd BulletDynamics_debug BulletDynamics_Debug
	HINTS
	${Bullet_LIBRARY_PATHS}
)

find_library(
	Bullet_BULLETDYNAMICS_LIBRARY_RELEASE
	NAMES
	bulletdynamics BulletDynamics
	HINTS
	${Bullet_LIBRARY_PATHS}
)

select_library_configurations(Bullet_BULLETDYNAMICS)

find_library(
	Bullet_BULLETSOFTBODY_LIBRARY_DEBUG
	NAMES
	bulletsoftbodyd BulletSoftBodyd BulletSoftBody_debug BulletSoftBody_Debug
	HINTS
	${Bullet_LIBRARY_PATHS}
)

find_library(
	Bullet_BULLETSOFTBODY_LIBRARY_RELEASE
	NAMES
	bulletsoftbody BulletSoftBody
	HINTS
	${Bullet_LIBRARY_PATHS}
)

select_library_configurations(Bullet_BULLETSOFTBODY)

find_library(
	Bullet_CONVEXDECOMPOSITION_LIBRARY_DEBUG
	NAMES
	convexdecompositiond ConvexDecompositiond ConvexDecomposition_debug ConvexDecomposition_Debug
	HINTS
	${Bullet_LIBRARY_PATHS}
)

find_library(
	Bullet_CONVEXDECOMPOSITION_LIBRARY_RELEASE
	NAMES
	convexdecomposition ConvexDecomposition
	HINTS
	${Bullet_LIBRARY_PATHS}
)

select_library_configurations(Bullet_CONVEXDECOMPOSITION)

find_library(
	Bullet_LINEARMATH_LIBRARY_DEBUG
	NAMES
	bulletmathd LinearMathd LinearMath_debug LinearMath_Debug
	HINTS
	${Bullet_LIBRARY_PATHS}
)

find_library(
	Bullet_LINEARMATH_LIBRARY_RELEASE
	NAMES
	bulletmath LinearMath
	HINTS
	${Bullet_LIBRARY_PATHS}
)

select_library_configurations(Bullet_LINEARMATH)

set(
	Bullet_LIBRARIES
	${Bullet_BULLETCOLLISION_LIBRARIES}
	${Bullet_BULLETDYNAMICS_LIBRARIES}
	${Bullet_BULLETSOFTBODY_LIBRARIES}
	${Bullet_LINEARMATH_LIBRARIES}
)

find_package_handle_standard_args(
	Bullet
	FOUND_VAR Bullet_FOUND
	REQUIRED_VARS Bullet_INCLUDE_DIRS Bullet_LIBRARIES
)

if((Bullet_BULLETCOLLISION_LIBRARY_RELEASE OR Bullet_BULLETCOLLISION_LIBRARY_DEBUG) AND Bullet_INCLUDE_DIRS)
	set(Bullet_BULLETCOLLISION_LIBRARY_FOUND ON)
endif()

if((Bullet_BULLETDYNAMICS_LIBRARY_RELEASE OR Bullet_BULLETDYNAMICS_LIBRARY_DEBUG) AND Bullet_INCLUDE_DIRS)
	set(Bullet_BULLETDYNAMICS_LIBRARY_FOUND ON)
endif()

if((Bullet_BULLETSOFTBODY_LIBRARY_RELEASE OR Bullet_BULLETSOFTBODY_LIBRARY_DEBUG) AND Bullet_INCLUDE_DIRS)
	set(Bullet_BULLETSOFTBODY_LIBRARY_FOUND ON)
endif()

if((Bullet_CONVEXDECOMPOSITION_LIBRARY_RELEASE OR Bullet_CONVEXDECOMPOSITION_LIBRARY_DEBUG) AND Bullet_INCLUDE_DIRS)
	set(Bullet_CONVEXDECOMPOSITION_LIBRARY_FOUND ON)
endif()

if((Bullet_LINEARMATH_LIBRARY_RELEASE OR Bullet_LINEARMATH_LIBRARY_DEBUG) AND Bullet_INCLUDE_DIRS)
	set(Bullet_LINEARMATH_LIBRARY_FOUND ON)
endif()

if(Bullet_BULLETCOLLISION_LIBRARY_FOUND AND NOT TARGET Bullet::BulletCollision)
	add_library(Bullet::BulletCollision UNKNOWN IMPORTED)
	
	if(Bullet_BULLETCOLLISION_LIBRARY_RELEASE)
		set_property(TARGET Bullet::BulletCollision APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Bullet::BulletCollision PROPERTIES IMPORTED_LOCATION_RELEASE "${Bullet_BULLETCOLLISION_LIBRARY_RELEASE}")
	endif()
	
	if(Bullet_BULLETCOLLISION_LIBRARY_DEBUG)
		set_property(TARGET Bullet::BulletCollision APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Bullet::BulletCollision PROPERTIES IMPORTED_LOCATION_DEBUG "${Bullet_BULLETCOLLISION_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Bullet::BulletCollision PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${Bullet_INCLUDE_DIRS}"
	)
endif()

if(Bullet_BULLETDYNAMICS_LIBRARY_FOUND AND NOT TARGET Bullet::BulletDynamics)
	add_library(Bullet::BulletDynamics UNKNOWN IMPORTED)
	
	if(Bullet_BULLETDYNAMICS_LIBRARY_RELEASE)
		set_property(TARGET Bullet::BulletDynamics APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Bullet::BulletDynamics PROPERTIES IMPORTED_LOCATION_RELEASE "${Bullet_BULLETDYNAMICS_LIBRARY_RELEASE}")
	endif()
	
	if(Bullet_BULLETDYNAMICS_LIBRARY_DEBUG)
		set_property(TARGET Bullet::BulletDynamics APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Bullet::BulletDynamics PROPERTIES IMPORTED_LOCATION_DEBUG "${Bullet_BULLETDYNAMICS_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Bullet::BulletDynamics PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${Bullet_INCLUDE_DIRS}"
	)
endif()

if(Bullet_BULLETSOFTBODY_LIBRARY_FOUND AND NOT TARGET Bullet::BulletSoftBody)
	add_library(Bullet::BulletSoftBody UNKNOWN IMPORTED)
	
	if(Bullet_BULLETSOFTBODY_LIBRARY_RELEASE)
		set_property(TARGET Bullet::BulletSoftBody APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Bullet::BulletSoftBody PROPERTIES IMPORTED_LOCATION_RELEASE "${Bullet_BULLETSOFTBODY_LIBRARY_RELEASE}")
	endif()
	
	if(Bullet_BULLETSOFTBODY_LIBRARY_DEBUG)
		set_property(TARGET Bullet::BulletSoftBody APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Bullet::BulletSoftBody PROPERTIES IMPORTED_LOCATION_DEBUG "${Bullet_BULLETSOFTBODY_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Bullet::BulletSoftBody PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${Bullet_INCLUDE_DIRS}"
	)
endif()

if(Bullet_CONVEXDECOMPOSITION_LIBRARY_FOUND AND NOT TARGET Bullet::ConvexDecomposition)
	add_library(Bullet::ConvexDecomposition UNKNOWN IMPORTED)
	
	if(Bullet_CONVEXDECOMPOSITION_LIBRARY_RELEASE)
		set_property(TARGET Bullet::ConvexDecomposition APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Bullet::ConvexDecomposition PROPERTIES IMPORTED_LOCATION_RELEASE "${Bullet_CONVEXDECOMPOSITION_LIBRARY_RELEASE}")
	endif()
	
	if(Bullet_CONVEXDECOMPOSITION_LIBRARY_DEBUG)
		set_property(TARGET Bullet::ConvexDecomposition APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Bullet::ConvexDecomposition PROPERTIES IMPORTED_LOCATION_DEBUG "${Bullet_CONVEXDECOMPOSITION_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Bullet::ConvexDecomposition PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${Bullet_INCLUDE_DIRS}"
	)
endif()

if(Bullet_LINEARMATH_LIBRARY_FOUND AND NOT TARGET Bullet::LinearMath)
	add_library(Bullet::LinearMath UNKNOWN IMPORTED)
	
	if(Bullet_LINEARMATH_LIBRARY_RELEASE)
		set_property(TARGET Bullet::LinearMath APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
		set_target_properties(Bullet::LinearMath PROPERTIES IMPORTED_LOCATION_RELEASE "${Bullet_LINEARMATH_LIBRARY_RELEASE}")
	endif()
	
	if(Bullet_LINEARMATH_LIBRARY_DEBUG)
		set_property(TARGET Bullet::LinearMath APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
		set_target_properties(Bullet::LinearMath PROPERTIES IMPORTED_LOCATION_DEBUG "${Bullet_LINEARMATH_LIBRARY_DEBUG}")
	endif()
	
	set_target_properties(
		Bullet::LinearMath PROPERTIES
		INTERFACE_INCLUDE_DIRECTORIES "${Bullet_INCLUDE_DIRS}"
	)
endif()
