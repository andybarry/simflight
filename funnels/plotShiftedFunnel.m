function plotShiftedFunnel(V,x0,options)

frameShifted = CoordinateFrame('shiftedFrame',length(x0));
frameShifted.addTransform(AffineTransform(frameShifted,V.getFrame,eye(length(x0)),-x0));
Vshifted = V.inFrame(frameShifted);
plotFunnel3(Vshifted,options);

end

