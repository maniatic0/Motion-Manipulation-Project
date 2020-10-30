module Transformation where

import Data.Geometry
import Data.Geometry.Matrix
import Data.Geometry.Transformation

-- | Create an identity transformation.
identity :: Transformation 2 Float
identity = Transformation . Matrix $ Vector3 (Vector3 1 0 0)
                                             (Vector3 0 1 0)
                                             (Vector3 0 0 1)

-- | Create a matrix that corresponds to a rotation by 'a' radians counter-clockwise 
--   around the origin.
rotation :: Float -> Transformation 2 Float
rotation a = Transformation . Matrix $ Vector3 (Vector3 (cos a) (- sin a) 0)
                                               (Vector3 (sin a) (  cos a) 0)
                                               (Vector3 0       0         1)

-- | Create a matrix that corresponds to a reflection in a line through the origin
--   which makes an angle of 'a' radians with the positive 'x'-asis, in counter-clockwise
--   orientation.
reflection :: Float -> Transformation 2 Float
reflection a = rotation a |.| reflectionV |.| rotation (-a)

reflectionV :: Transformation 2 Float
reflectionV = Transformation . Matrix $ Vector3 (Vector3 1   0  0)
                                                (Vector3 0 (-1) 0)
                                                (Vector3 0   0  1)

reflectionH :: Transformation 2 Float
reflectionH = Transformation . Matrix $ Vector3 (Vector3 (-1) 0  0)
                                                (Vector3   0  1  0)
                                                (Vector3   0  0  1)

