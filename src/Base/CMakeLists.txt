
# @author Shin'ichiro Nakaoka

# set(CMAKE_BUILD_TYPE Debug)

set(sources
  App.cpp
  ItemManager.cpp
  MenuManager.cpp
  ProjectManager.cpp
  ProjectPathSetEditor.cpp
  PluginManager.cpp
  MainWindow.cpp
  ToolBarArea.cpp
  InfoBar.cpp
  View.cpp
  MessageView.cpp
  SceneView.cpp
  OsgViewer.cpp
  ItemTreeView.cpp
  ItemPropertyView.cpp
  MultiSeqItemCreationPanel.cpp
  ToolBar.cpp
  TimeBar.cpp
  FileBar.cpp
  GraphWidget.cpp
  GraphBar.cpp
  Action.cpp
  SpinBox.cpp
  Slider.cpp
  ScrollBar.cpp
  Button.cpp
  ButtonGroup.cpp
  LineEdit.cpp
  ComboBox.cpp
  Dialog.cpp
  DescriptionDialog.cpp
  TreeView.cpp
  TreeWidget.cpp
  ItemSelectionModel.cpp
  ExtensionManager.cpp
  OptionManager.cpp
  Plugin.cpp
  Item.cpp
  RootItem.cpp
  FolderItem.cpp
  SceneItem.cpp
  MultiSeqItem.cpp
  MultiValueSeqItem.cpp
  MultiAffine3SeqItem.cpp
  Vector3SeqItem.cpp
  ItemPath.cpp
  TimeSyncItemEngineManager.cpp
  AppConfig.cpp
  Archive.cpp
  ItemTreeArchiver.cpp
  SceneObject.cpp
  SceneGL.cpp
  ScenePieces.cpp
  ConnectionSet.cpp
  LazyCaller.cpp
  LazySignal.cpp
  OsgNormalVisualizer.cpp
  OsgOutlineFx.cpp
  VrmlToOsgConverter.cpp
  Licenses.cpp
  )

set(headers
  App.h
  MainWindow.h
  Action.h
  SpinBox.h
  Slider.h
  ScrollBar.h
  Button.h
  ButtonGroup.h
  LineEdit.h
  ComboBox.h
  Dialog.h
  TreeView.h
  TreeWidget.h
  ItemSelectionModel.h
  MenuManager.h
  ToolBar.h
  View.h
  OsgViewer.h
  GraphWidget.h
  ExtensionManager.h
  OptionManager.h
  ProjectManager.h
  PluginManager.h
  Plugin.h
  MessageView.h
  SceneView.h
  ItemTreeView.h
  ItemList.h
  Item.h
  PutPropertyFunction.h
  RootItem.h
  FolderItem.h
  SceneItem.h
  MultiSeqItem.h
  MultiValueSeqItem.h
  MultiAffine3SeqItem.h
  Vector3SeqItem.h
  ItemPath.h
  TimeBar.h
  TimeSyncItemEngineManager.h
  GraphBar.h
  ItemManager.h
  AppConfig.h
  Archive.h
  ItemTreeArchiver.h
  SceneObject.h
  SceneGL.h
  ScenePieces.h
  ConnectionSet.h
  SignalProxy.h
  LazySignal.h
  LazyCaller.h
  VrmlToOsgConverter.h
  exportdecl.h
  gettext.h
  )

QT4_WRAP_CPP(sources
  MenuManager.h
  ToolBar.h
  Action.h
  SpinBox.h
  Slider.h
  ScrollBar.h
  Button.h
  ButtonGroup.h
  ComboBox.h
  LineEdit.h
  Dialog.h
  TreeView.h
  TreeWidget.h
  ItemSelectionModel.h
  InfoBar.h
  ItemTreeView.h
  )

QT4_ADD_RESOURCES(RC_SRCS Base.qrc)

set(target CnoidBase)
make_gettext_mofiles(mofiles)
add_library(${target} SHARED ${sources} ${headers} ${mofiles} ${RC_SRCS})


if(APPLE)

  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY}
    intl osg osgDB osgUtil osgViewer osgGA osgManipulator osgText osgFX osgShadow OpenThreads GL
    )

elseif(UNIX)

  #if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  #  set_source_files_properties(VrmlToOsgConverter.cpp PROPERTIES COMPILE_FLAGS "-O3")
  #endif()
  target_link_libraries(${target}
    CnoidUtil
    ${QT_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY}
    osg osgDB osgUtil osgViewer osgGA osgManipulator osgText osgFX osgShadow OpenThreads GL icuuc
    )

elseif(MSVC)

  target_link_libraries(${target}
    ${QT_LIBRARIES}
    optimized osg debug osgd
    optimized osgDB debug osgDBd
    optimized osgUtil debug osgUtild
    optimized osgViewer debug osgViewerd
    optimized osgGA debug osgGAd
    optimized osgManipulator debug osgManipulatord
    optimized osgText debug osgTextd
    optimized osgFX debug osgFXd
    optimized osgShadow debug osgShadowd
    optimized OpenThreads debug OpenThreadsd
    general opengl32 glu32
    CnoidUtil
    )
endif()

apply_common_setting_for_library(${target} "${headers}")


install_external_libraries(${QT_BINARY_DIR} ${QT_LIBRARY_DIR} ${QT_LIBRARIES})
install_external_libraries(${Boost_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${Boost_PROGRAM_OPTIONS_LIBRARY})


# Installing OpenSceneGraph

if(MSVC AND INSTALL_DEPENDENCIES)

  foreach(library osg osgDB osgUtil osgViewer osgGA osgManipulator osgText osgFX osgShadow OpenThreads)
    file(GLOB dllfile "${OSG_DIR}/bin/*${library}.dll")
    install(PROGRAMS ${dllfile} DESTINATION bin CONFIGURATIONS Release)
    if(INSTALL_SDK)
      install(FILES ${OSG_DIR}/lib/${library}.lib DESTINATION lib CONFIGURATIONS Release)
    endif()
    file(GLOB dllfile "${OSG_DIR}/bin/*${library}d.dll")
    install(PROGRAMS ${dllfile} DESTINATION bin CONFIGURATIONS Debug)
    if(INSTALL_SDK)
      install(FILES ${OSG_DIR}/lib/${library}d.lib DESTINATION lib CONFIGURATIONS Debug)
    endif()
  endforeach()

  file(GLOB pluginpath "${OSG_DIR}/bin/osgPlugins-*")
  if(pluginpath)
    file(RELATIVE_PATH pluginfolder ${OSG_DIR} ${pluginpath})
    foreach(plugin osgdb_3ds osgdb_dxf osgdb_lwo osgdb_osg osgdb_ply osgdb_pov osgdb_x osgdb_bvh)
      file(GLOB dllfile "${OSG_DIR}/${pluginfolder}/${plugin}.dll")
      install(PROGRAMS ${dllfile} DESTINATION ${pluginfolder} CONFIGURATIONS Release)
      file(GLOB dllfile "${OSG_DIR}/${pluginfolder}/${plugin}d.dll")
      install(PROGRAMS ${dllfile} DESTINATION ${pluginfolder} CONFIGURATIONS Debug)
    endforeach()
  endif()
endif()
