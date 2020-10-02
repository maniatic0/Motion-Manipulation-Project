module PingPong.Model where


import Data.Geometry hiding (init)
import Data.Geometry.Matrix
import Data.Geometry.Transformation
import Data.Geometry.PolyLine
import Data.Geometry.Polygon

import Data.Ext
import Control.Lens

import Graphics.Gloss (Color)

data Element = Link Color Float
             | Joint Color Float

type Arm = [Element]

data BallState = BallState
  { loc :: Point 2 Float
  , dir :: Vector 2 Float
  }

data Player = Player 
  { name    :: String  
  , arm     :: Arm
  , foot    :: Float
  , action  :: BallState -> Arm -> IO Motion
  , collide :: (Float, Point 2 Float, LineSegment 2 () Float) 
            -> (Float, Point 2 Float, LineSegment 2 () Float) 
            -> Point 2 Float
  , planPnt :: Float -> Arm -> Point 2 Float -> [Float]
  , planSeg :: Float -> Arm -> LineSegment 2 () Float -> [Float]
  }


type Motion = [Float] -- speed in radians per second at which joints should move




data State = State
  { p1, p2 :: Player
  , ball   :: BallState
  -- TODO: current score, whose turn is it, what was the last thing the ball hit
  }

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

transformation :: Element -> Transformation 2 Float
transformation (Link _ d) = translation $ Vector2 0 d
transformation (Joint _ a) = rotation a

transformations :: Arm -> [Transformation 2 Float]
transformations = map transformation

globalize :: [Transformation 2 Float] -> [Transformation 2 Float]
globalize ts = scanl (|.|) identity ts

evaluate :: Arm -> [Point 2 Float]
evaluate arm = map (origin .+^) 
             $ map (flip transformBy $ Vector2 0 0) 
             $ globalize $ transformations arm

evaluateP :: Arm -> PolyLine 2 () Float
evaluateP arm = fromPointsUnsafe
              $ map (:+ ())
              $ evaluate arm
 

room :: SimplePolygon () Float
room = Data.Geometry.Polygon.fromPoints $ map (:+ ()) [Point2 (-3) 0, Point2 3 0, Point2 3 6, Point2 (-3) 6]

table :: SimplePolygon () Float
table = Data.Geometry.Polygon.fromPoints $ map (:+ ()) [Point2 (-1) 0.4, Point2 1 0.4, Point2 1 0.5, Point2 (-1) 0.5]

net :: LineSegment 2 () Float
net = ClosedLineSegment (Point2 0 0.5 :+ ()) (Point2 0 0.6 :+ ())


armSegments :: Player -> Bool -> [LineSegment 2 () Float]
armSegments p f = 
  let rps = evaluate $ arm p
      trl = translation (Vector2 (foot p) 0)
      scl = scaling (Vector2 (if f then 1 else -1) 1)
      nps = map (transformBy (scl |.| trl)) rps
      fps = map (:+ ()) nps
  in zipWith (OpenLineSegment) (init fps) (tail fps)


