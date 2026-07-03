-- Currently uses Figura's vector library. Can be swapped for another.

-- This just adds Figura's dot and cross product as metamethods
figuraMetatables.Vector3.__concat = vec(0,0,0).dot
figuraMetatables.Vector3.__pow = vec(0,0,0).crossed


-- vec = function(x,y,z) return vec(x,y,z) end