# Install rules and installers (CPack):
#   cmake --build build/release && cd build/release && cpack
# writes, per platform:
#   Windows  NSIS installer (.exe, Start-menu shortcut, uninstaller) and a .zip
#   macOS    drag-to-Applications disk image (.dmg)
#   Linux    .tar.gz and .deb (with a desktop entry and icons)

include(GNUInstallDirs)

if(APPLE)
    set(SICDM_BIN_DIR ".")
    set(SICDM_DOC_DIR "SI CDM Documentation")
elseif(WIN32)
    set(SICDM_BIN_DIR "bin")
    set(SICDM_DOC_DIR "doc")
else()
    set(SICDM_BIN_DIR "${CMAKE_INSTALL_BINDIR}")
    set(SICDM_DOC_DIR "${CMAKE_INSTALL_DATAROOTDIR}/doc/sicdm")
endif()

if(TARGET sicdm)
    if(APPLE)
        install(TARGETS sicdm BUNDLE DESTINATION "." COMPONENT app)
    else()
        install(TARGETS sicdm RUNTIME DESTINATION "${SICDM_BIN_DIR}" COMPONENT app)
    endif()
endif()
install(TARGETS sicdm_mcp RUNTIME DESTINATION "${SICDM_BIN_DIR}" COMPONENT app)

install(FILES README.md resources/THIRD-PARTY-NOTICES.txt DESTINATION "${SICDM_DOC_DIR}" COMPONENT app)
install(DIRECTORY docs/ DESTINATION "${SICDM_DOC_DIR}/docs" COMPONENT app)
install(FILES resources/fonts/OFL-Inter.txt resources/fonts/OFL-JetBrainsMono.txt third_party/stb/README.txt
        DESTINATION "${SICDM_DOC_DIR}/licenses" COMPONENT app)
if(SICDM_BUILD_APP)
    install(FILES "${glfw_SOURCE_DIR}/LICENSE.md" DESTINATION "${SICDM_DOC_DIR}/licenses" RENAME GLFW-LICENSE.md COMPONENT app)
    install(FILES "${imgui_SOURCE_DIR}/LICENSE.txt" DESTINATION "${SICDM_DOC_DIR}/licenses" RENAME ImGui-LICENSE.txt COMPONENT app)
endif()

if(UNIX AND NOT APPLE)
    install(FILES resources/linux/sicdm.desktop DESTINATION "${CMAKE_INSTALL_DATADIR}/applications" COMPONENT app)
    install(FILES resources/linux/sicdm-mime.xml DESTINATION "${CMAKE_INSTALL_DATADIR}/mime/packages" RENAME sicdm.xml COMPONENT app)
    foreach(size 32 64 256)
        install(FILES resources/icon/sicdm_${size}.png DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/${size}x${size}/apps"
                RENAME sicdm.png COMPONENT app)
    endforeach()
endif()

set(CPACK_PACKAGE_NAME "sicdm")
set(CPACK_PACKAGE_VENDOR "SI CDM")
set(CPACK_PACKAGE_CONTACT "SI CDM <support@sicdm.invalid>")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "SI CDM")
set(CPACK_PACKAGE_EXECUTABLES "sicdm;SI CDM")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/resources/THIRD-PARTY-NOTICES.txt")
set(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
set(CPACK_STRIP_FILES ON)
set(CPACK_VERBATIM_VARIABLES ON)

if(WIN32)
    set(CPACK_GENERATOR "NSIS;ZIP")
    set(CPACK_PACKAGE_FILE_NAME "SICDM-${PROJECT_VERSION}-windows-x64")
    set(CPACK_NSIS_PACKAGE_NAME "SI CDM ${PROJECT_VERSION}")
    set(CPACK_NSIS_DISPLAY_NAME "SI CDM")
    set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}/resources/windows/sicdm.ico")
    set(CPACK_NSIS_MUI_UNIICON "${PROJECT_SOURCE_DIR}/resources/windows/sicdm.ico")
    set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\sicdm.exe")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    set(CPACK_NSIS_MODIFY_PATH OFF)
    set(CPACK_NSIS_CREATE_ICONS_EXTRA "CreateShortCut '$DESKTOP\\\\SI CDM.lnk' '$INSTDIR\\\\bin\\\\sicdm.exe'")
    set(CPACK_NSIS_DELETE_ICONS_EXTRA "Delete '$DESKTOP\\\\SI CDM.lnk'")
    # .sicdm projects open in SI CDM (double-click, Open With). The uninstaller takes the
    # association away only while it is still ours.
    set(CPACK_NSIS_EXTRA_INSTALL_COMMANDS "
        WriteRegStr SHCTX 'Software\\Classes\\.sicdm' '' 'SICDM.Project'
        WriteRegStr SHCTX 'Software\\Classes\\.sicdm' 'Content Type' 'application/x-sicdm'
        WriteRegStr SHCTX 'Software\\Classes\\SICDM.Project' '' 'SI CDM Project'
        WriteRegStr SHCTX 'Software\\Classes\\SICDM.Project\\DefaultIcon' '' '$INSTDIR\\bin\\sicdm.exe,0'
        WriteRegStr SHCTX 'Software\\Classes\\SICDM.Project\\shell\\open\\command' '' '\"$INSTDIR\\bin\\sicdm.exe\" \"%1\"'
        System::Call 'shell32::SHChangeNotify(i 0x08000000, i 0, p 0, p 0)'")
    set(CPACK_NSIS_EXTRA_UNINSTALL_COMMANDS "
        ReadRegStr $0 SHCTX 'Software\\Classes\\.sicdm' ''
        StrCmp $0 'SICDM.Project' 0 +2
        DeleteRegKey SHCTX 'Software\\Classes\\.sicdm'
        DeleteRegKey SHCTX 'Software\\Classes\\SICDM.Project'
        System::Call 'shell32::SHChangeNotify(i 0x08000000, i 0, p 0, p 0)'")
elseif(APPLE)
    set(CPACK_GENERATOR "DragNDrop")
    set(CPACK_PACKAGE_FILE_NAME "SICDM-${PROJECT_VERSION}-macos")
    set(CPACK_DMG_VOLUME_NAME "SI CDM ${PROJECT_VERSION}")
    set(CPACK_DMG_FORMAT "UDBZ")
else()
    set(CPACK_GENERATOR "TGZ;DEB")
    set(CPACK_PACKAGE_FILE_NAME "sicdm-${PROJECT_VERSION}-linux-${CMAKE_SYSTEM_PROCESSOR}")
    set(CPACK_DEBIAN_FILE_NAME "DEB-DEFAULT")
    set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/srinivasan-fh/si-cdm")
    find_program(SICDM_DPKG_SHLIBDEPS dpkg-shlibdeps)
    if(SICDM_DPKG_SHLIBDEPS)
        set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
    endif()
endif()

include(CPack)
