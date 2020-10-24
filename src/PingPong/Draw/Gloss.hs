module PingPong.Draw.Gloss where

import PingPong.Model
import PingPong.Draw

import Graphics.Gloss (Picture, Color)
import qualified Graphics.Gloss as G

import Data.Geometry hiding (head)

import Convert

drawState :: State -> Picture
drawState m = G.Pictures 
  [ center $ G.Pictures [drawStatic, drawPlayer (p1 m) True, drawPlayer (p2 m) False, drawBall $ ball m]
  , drawInfo m
  , drawPhase (phase m) m
  ]

drawInfo :: State -> Picture
drawInfo s = G.Pictures
  [ G.Translate (-580) (320) $ G.Scale 0.1 0.1 $ G.text $ (name $ p2 $ s) ++ " VS " ++ (name $ p1 $ s)
  , G.Translate (-580) (300) $ G.Scale 0.1 0.1 $ G.text $ "score: " ++ (show $ snd $ score s) ++ " - " ++ (show $ fst $ score s)
  , G.Translate (-580) (280) $ G.Scale 0.1 0.1 $ G.text $ "time: " ++ (dec $ time s)
  , G.Translate (-580) (260) $ G.Scale 0.1 0.1 $ G.text $ "FPS: " ++ (dec $ (fromInteger $ toInteger $ frame s) / time s)
  , G.Translate (-580) (240) $ G.Scale 0.1 0.1 $ G.text $ "last hit: " ++ (show $ snd $ head $ hits s)
  , G.Translate (-580) (220) $ G.Scale 0.1 0.1 $ G.text $ "phase: " ++ (show $ phase s)
  ]

drawPhase :: Phase -> State -> Picture
drawPhase DuringRally s = G.Blank
drawPhase (BeforeGame _) s = G.Translate (-580) (100) $ G.Scale 1 1 $ G.text $ (name $ p2 $ s) ++ " VS " ++ (name $ p1 $ s)
drawPhase (BeforeRally _) s = G.Translate (-580) (100) $ G.Scale 1 1 $ G.text $ "prepare for action"
drawPhase (AfterRally _) s = G.Translate (-280) (100) $ G.Scale 1 1 $ G.text $ (show $ snd $ score s) ++ " - " ++ (show $ fst $ score s)
drawPhase (AfterGame _) s = G.Translate (-580) (100) $ G.Scale 1 1 $ G.text $ (name $ winner $ s) ++ " wins!"

dec :: Float -> String
dec x = show $ (fromInteger $ floor $ 100 * x) / 100

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
