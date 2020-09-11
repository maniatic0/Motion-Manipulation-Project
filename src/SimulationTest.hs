import Graphics.Gloss
import Graphics.Gloss.Data.ViewPort

windowDisplay :: Display
windowDisplay = InWindow "Window" (200, 200) (10, 10)

--main = animate windowDisplay white animationFunc

animationFunc :: Float -> Picture
animationFunc time = Circle (2 * time)

type Model = (Float, Float)

main = simulate
  windowDisplay
  white
  simulationRate
  initialModel
  drawingFunc
  updateFunc
  where
    simulationRate :: Int
    simulationRate = 20

    initialModel :: Model
    initialModel = (0,0)

    drawingFunc :: Model -> Picture
    drawingFunc (theta, dtheta) = Line [(0, 0), (50 * cos theta, 50 * sin theta)]

    updateFunc :: ViewPort -> Float -> Model -> Model
    updateFunc _ dt (theta, dtheta) = (theta + dt * dtheta, dtheta - dt * (cos theta))
