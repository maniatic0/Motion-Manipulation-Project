module PingPong.Draw.Gloss where

import PingPong.Model
import PingPong.Draw

import Graphics.Gloss (Picture, Color)
import qualified Graphics.Gloss as G

import Data.Geometry

import Convert

drawState :: State -> Picture
drawState m = G.Pictures 
  [ center $ G.Pictures [drawStatic, drawPlayer (p1 m) True, drawPlayer (p2 m) False, drawBall $ ball m]
  , drawInfo m
  ]

drawInfo :: State -> Picture
drawInfo s = G.Pictures
  [ G.Translate (-580) (320) $ G.Scale 0.1 0.1 $ G.text $ (name $ p1 $ s) ++ " VS " ++ (name $ p2 $ s)
  , G.Translate (-580) (300) $ G.Scale 0.1 0.1 $ G.text $ "time: " ++ (show $ (fromInteger $ floor $ 100 * time s) / 100)
  , G.Translate (-580) (280) $ G.Scale 0.1 0.1 $ G.text $ "last hit: " ++ (show $ snd $ hit s)
  ]



center :: Picture -> Picture
center = G.Scale 200 200 . G.Translate 0 (-1)

toFloat :: RealFrac r => Point 2 r -> (Float, Float)
toFloat (Point2 x y) = (realToFrac x, realToFrac y)

drawAt :: RealFrac r => Point 2 r -> Picture -> Picture
drawAt p = uncurry G.Translate (toFloat p)


drawStatic :: Picture
drawStatic = G.Pictures [drawRoom, drawTable, drawNet]

drawRoom :: Picture
drawRoom = G.Color cRoom $ glossify room -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawTable :: Picture
drawTable = G.Color cTable $ glossify table -- Line [(-1, 0), (1, 0), (1, 0.5), (-1, 0.5)]

drawNet :: Picture
drawNet = G.Color cNet $ glossify net -- Line [(0, 0.5), (0, 0.6)]

drawBall :: BallState -> Picture
drawBall s = drawAt (loc s) $ G.Color cBall $ G.circleSolid 0.02

drawPlayer :: Player -> Bool -> Picture
--drawPlayer p b = G.Pictures $ map glossify $ armSegments p b

drawPlayer p True  =                  G.Translate (foot p) 0 $ drawArm $ arm p -- possibly flip around 0
drawPlayer p False = G.Scale (-1) 1 $ G.Translate (foot p) 0 $ drawArm $ arm p -- possibly flip around 0


drawArm :: Arm -> Picture
--drawArm a = glossify $ evaluateP a

drawArm a = drawArm' a $ evaluate a
  where
    drawArm' :: Arm -> [Point 2 Float] -> Picture
    drawArm' (Link  l _ : rest) (p : q : qs) = G.Pictures [drawLink p q l, drawArm' rest $ q : qs]
    drawArm' (Joint j _ : rest) (p :     ps) = G.Pictures [drawArm' rest ps, drawJoint p j]
    drawArm' _ _ = G.Blank


drawLink :: Point 2 Float -> Point 2 Float -> Color -> Picture
drawLink p q c = G.Color c $ thickLine 0.01 $ map glossify [p, q]

drawJoint :: Point 2 Float -> Color -> Picture
drawJoint p c = G.Color c $ drawAt p $ G.circleSolid 0.01
