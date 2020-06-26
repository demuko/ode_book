project.name = "roboSimu"


-- We no longer support VC++ 6.0; too many incompatibilities

  if (options["target"] == "vs6") then
    error("Visual Studio 6 is no longer supported; please upgrade to Visual Studio 2005 C++ Express.")
  end


-- Define the build configurations. You can also use the flags
-- `--enable-shared-only` and `--enable-static-only` if you want to
-- call these packages from within your own Premake-enabled project.

  if (options["enable-shared-only"]) then
    project.configs = { "DebugDLL" }
  elseif (options["enable-static-only"]) then
    project.configs = { "DebugLib"}
  else
    project.configs = {"DebugLib","DebugDLL" }
  end


-- Project options

--  addoption("with-demos",    		"Builds the demo applications and DrawStuff library")
--  addoption("with-tests",    		"Builds the unit test application")
--  addoption("with-gimpact",  		"Use GIMPACT for trimesh collisions (experimental)")
  addoption("enable-static-only",	"Only create static library (.lib) project configurations")
  addoption("enable-shared-only",	"Only create dynamic library (.dll) project configurations")
--  addoption("no-dif",        		"Exclude DIF (Dynamics Interchange Format) exports")
--  addoption("no-trimesh",    		"Exclude trimesh collision geometry")
--  addoption("no-alloca",     		"Use heap memory instead of the stack (experimental)")
--  addoption("enable-ou",            "Use TLS for global variables (experimental)")

  -- Output is placed in a directory named for the target toolset.
--  project.path = options["target"]
  project.path = "src"


-- Set the output directories

if (not options["enable-shared-only"]) then

--    project.config["DebugLib"].bindir   = "../lib/DebugDoubleLib"
--    project.config["DebugLib"].libdir   = "../lib/DebugDoubleLib"
--    project.config["ReleaseLib"].bindir = "../lib/ReleaseDoubleLib"
--    project.config["ReleaseLib"].libdir = "../lib/ReleaseDoubleLib"
    project.config["DebugLib"].bindir   = "../roboSimu/DebugLib"
    project.config["DebugLib"].libdir   = "../roboSimu/DebugLib"
--    project.config["ReleaseLib"].bindir = "../roboSimu/ReleaseLib"
--    project.config["ReleaseLib"].libdir = "../roboSimu/ReleaseLib"

  end



  if (not options["enable-static-only"]) then

--		project.config["DebugDLL"].bindir   = "../lib/DebugDoubleDLL"
--    project.config["DebugDLL"].libdir   = "../lib/DebugDoubleDLL"
--    project.config["ReleaseDLL"].bindir = "../lib/ReleaseDoubleDLL"
--    project.config["ReleaseDLL"].libdir = "../lib/ReleaseDoubleDLL"
		project.config["DebugDLL"].bindir   = "../roboSimu/DebugDLL"
    project.config["DebugDLL"].libdir   = "../roboSimu/DebugDLL"
--    project.config["ReleaseDLL"].bindir = "../roboSimu/ReleaseDLL"
--    project.config["ReleaseDLL"].libdir = "../roboSimu/ReleaseDLL"

  end




-- Build packages

  dopackage("roboSimu.lua")

-- Remove all intermediate files

  function doclean(cmd, arg)
    docommand(cmd, arg)
    if (options["target"] == "") then
      os.remove("../ode/src/config.h")
    end
    os.rmdir("custom")
		os.rmdir("cb-gcc/obj")
    os.rmdir("gnu/obj")
    os.rmdir("vs2002/obj")
    os.rmdir("vs2003/obj")
    os.rmdir("vs2005/obj")
  end


-- Generate all toolsets in one go

  function domakeall(cmd, arg)
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target vs2002")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target vs2003")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target vs2005")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target gnu")
    os.execute("premake --usetargetpath --with-demos --with-tests --clean --target cb-gcc")

  end
