-- Here are the lists of demos to build. Add/remove new
-- demos here and everything else should just work

  local demos =
  {
    "amotor",
		"arm1",
		"arm2",
		"arm3",
		"bounce",
		"hello",
		"hopper",
		"hopper2",
		"hopper3",
		"legged",
		"monoBot",
		"omni",
		"pk",
		"sensor4",
		"slope",
		"wheel1",
		"wheel2",
		"wheel4"
  }


  -- Output is placed in a directory named for the target toolset.
  --packagepath = options["target"]
 packagepath = "src"


-- Factory function for demo packages

  function makedemo(index, name)
    package = newpackage()
    package.name = "" .. name
    package.kind = "exe"
    package.language = "c++"
    package.path = packagepath
    package.objdir = "obj/"..name
    --package.objdir = "../../roboSimu/src/obj/"..name


   table.insert(package.config["DebugLib"].libpaths, {"../../lib/Debug"})
	 table.insert(package.config["DebugDLL"].libpaths, {"../../lib/Debug"})

--   table.insert(package.config["ReleaseDLL"].libpaths, {"../../lib/ReleaseDoubleDLL"})
--	 table.insert(package.config["ReleaseLib"].libpaths, {"../../lib/ReleaseDoubleLib"})



    package.includepaths =
    {
    "../../include",
    "../../ode/src"
    }
    package.defines = { "_CRT_SECURE_NO_DEPRECATE" }

    if (options.target == "vs6" or options.target == "vs2002" or options.target == "vs2003") then
      package.config.DebugLib.buildflags   = { "static-runtime" }
      package.config.ReleaseLib.buildflags = { "static-runtime" }
    end

 	 package.config.DebugDLL.links =  {"oded", "drawstuffd"}
	 package.config.DebugLib.links =  {"oded", "drawstuffd"}

-- 	 package.config.ReleaseDLL.links =  {"ode_double", "drawstuff"}
--	 package.config.ReleaseLib.links =  {"ode_double", "drawstuff"}


    package.links = { "oded", "drawstuffd" }
--   package.links = { "ode_double", "drawstuff" }

     if (windows) then
      table.insert(package.links, { "user32", "winmm", "gdi32", "opengl32", "glu32" })
    else
      table.insert(package.links, { "GL", "GLU" })
    end

    if (name == "chain1") then
      package.files = { "../../roboSimu/src/" .. name .. ".c" }
    else
      package.files = { "../../roboSimu/src/" .. name .. ".cpp" }
    end

    if (windows) then
      table.insert(package.defines, "WIN32")
      table.insert(package.files, "../../drawstuff/src/resources.rc")
   end

   if (not options["enable-static-only"]) then
		table.insert(package.config["DebugLib"].defines, "ODE_LIB")
 --   table.insert(package.config["ReleaseLib"].defines, "ODE_LIB")
    table.insert(package.config["DebugDLL"].defines, "dDOUBLE")
 ---  table.insert(package.config["ReleaseDLL"].defines, "dDOUBLE")
   end
   if (not options["enable-shared-only"]) then
 		table.insert(package.config["DebugDLL"].defines, "ODE_DLL")
 --   table.insert(package.config["ReleaseDLL"].defines, "ODE_DLL")
    table.insert(package.config["DebugLib"].defines, "dDOUBLE")
 --   table.insert(package.config["ReleaseLib"].defines, "dDOUBLE")
   end

	package.config["DebugDLL"].buildflags   = { }
	package.config["DebugLib"].buildflags   = { }


--	package.config["ReleaseDLL"].buildflags = { "optimize-speed", "no-symbols", "no-frame-pointer" }
--	package.config["ReleaseLib"].buildflags = { "optimize-speed", "no-symbols", "no-frame-pointer" }


  end

  table.foreach(demos, makedemo)
