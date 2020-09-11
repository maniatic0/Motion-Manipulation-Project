import Graphics.Gloss
import Graphics.Gloss.Data.ViewPort

import Graphics.Gloss.Interface.Pure.Game

windowDisplay :: Display
windowDisplay = InWindow "Window" (200, 200) (10, 10)

type World = (Float, Float)

main :: IO ()
main = play
  windowDisplay
  white
  20
  (0, 0)
  drawingFunc
  inputHandler
  updateFunc

drawingFunc :: World -> Picture
drawingFunc (x, y) = translate x y (Circle 20)

inputHandler :: Event -> World -> World
inputHandler (EventKey (SpecialKey KeyUp) Down _ _) (x, y) = (x, y + 10)
inputHandler (EventKey (SpecialKey KeyDown) Down _ _) (x, y) = (x, y - 10)
inputHandler (EventKey (SpecialKey KeyRight) Down _ _) (x, y) = (x + 10, y)
inputHandler (EventKey (SpecialKey KeyLeft) Down _ _) (x, y) = (x - 10, y)
inputHandler _ w = w

updateFunc :: Float -> World -> World
updateFunc _ (x, y) = (towardCenter x, towardCenter y)
  where
    towardCenter :: Float -> Float
    towardCenter c = if abs c < 0.25
      then 0
      else if c > 0
        then c - 0.25
        else c + 0.25
