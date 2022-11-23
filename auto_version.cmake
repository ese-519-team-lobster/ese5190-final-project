# from https://stackoverflow.com/questions/35745344/cmake-target-version-increment
message("Updating version...")
#basic definitions
set(HEADER_FILE "${GEN_DIR}/proj_version.h")
set(CACHE_FILE "${GEN_DIR}/BuildNumberCache.txt")
message("Looking for ${GEN_DIR}/proj_version.h")
#Reading data from file + incrementation
IF(EXISTS ${CACHE_FILE})
    file(READ ${CACHE_FILE} INCREMENTED_VALUE)
    math(EXPR INCREMENTED_VALUE "${INCREMENTED_VALUE}+1")
ELSE()
    set(INCREMENTED_VALUE "1")
ENDIF()

#Update the cache
file(WRITE ${CACHE_FILE} "${INCREMENTED_VALUE}")

#Create the header
file(WRITE ${HEADER_FILE} "#ifndef PROJ_VERSION_H\n#define PROJ_VERSION_H\n\n#define PROJECT_VERSION ${INCREMENTED_VALUE}\n\n#endif")