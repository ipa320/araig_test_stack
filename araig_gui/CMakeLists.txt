cmake_minimum_required(VERSION 3.0.2)
project(araig_gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
  nodelet
  araig_msgs
)


if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs araig_msgs
)


set (araig_gui_SRCS
  src/araig_gui/araig_gui.cpp
)

set(araig_gui_HDRS
  include/araig_gui/araig_gui.h
)

set(araig_gui_UIS
  src/araig_gui/araig_gui.ui
)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(araig_gui_MOCS ${araig_gui_HDRS})
  qt5_wrap_ui(araig_gui_UIS_H ${araig_gui_UIS})
else()
  qt4_wrap_cpp(araig_gui_MOCS ${araig_gui_HDRS})
  qt4_wrap_ui(araig_gui_UIS_H ${araig_gui_UIS})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}
)

add_library(${PROJECT_NAME}
    #${araig_gui_SRCS}
    "src/araig_gui/araig_gui.cpp"
    ${araig_gui_MOCS}
    ${araig_gui_UIS_H}
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


install(PROGRAMS
  scripts/araig_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  test/test_araig_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
