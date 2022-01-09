docker run --name "build_sesc" --rm --privileged --pid=host --mount type=bind,source=/${PWD},target=//project wsbu/stm32cubeide bash -c "//opt/stm32cubeide/stm32cubeide --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build $1 -importAll //project"

