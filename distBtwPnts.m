function isEqualBool = distBtwPnts(x1, y1, z1, x2, y2, z2, Distance)
    value = sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2)
    if (value < (Distance+ 0.00001) & value > (Distance - 0.00001))
        isEqualBool = true
    else
        isEqualBool = false
    end
end