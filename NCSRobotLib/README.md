# NCSRobotLib
Library to interface the MadBot, PushBot and Kepera at INI.

# Installation
To simply use this library with your project, add it as a submodule to your git repository:

    git submodule init
    git submodule add <url here>

In addition, you have to link the library into your project by updating your `CMakeList.txt`, following this example:

    #add RobotLib
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/../NCSRobotLib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../NCSRobotLib/aerctl)
    link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../NCSRobotLib/build)
    
    # build executables
    add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
    target_link_libraries(${PROJECT_NAME}
                          ${PROJECT_NAME}_lib
                          Robot-lib)

## Update
To update the library to the newest version, simply type

    git submodule update --remote

## Customization
You may need to make custom changes for your specific project. As this library needs to stay stable for all users, we recommend you to **fork this repository** and add your own fork as submodule as described above.  
We strongly recommend you to contribute to this common library by proposing **pull requests** with your new features.
