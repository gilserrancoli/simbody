# add in local source and headers here
# TODOautodiff file(GLOB src_files  ./src/*.cpp)
file(GLOB src_files  ./src/Contact.cpp
                     ./src/gcvspl.cpp
                     ./src/Geo_Box.cpp
                     ./src/Geo_Sphere.cpp
                     ./src/GCVSPLUtil.cpp
                     ./src/OBBTree.cpp)
file(GLOB incl_files ./src/*.h)

# append to the local scope copy, and then copy up to parent scope
list(APPEND SOURCE_FILES ${src_files})
set(SOURCE_FILES ${SOURCE_FILES} PARENT_SCOPE)

list(APPEND SOURCE_INCLUDE_FILES ${incl_files})
set(SOURCE_INCLUDE_FILES ${SOURCE_INCLUDE_FILES} PARENT_SCOPE)
