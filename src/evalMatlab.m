rosshutdown
rosinit
sub = rossubscriber('/omni1EstimationError')
vec = []
while 1
    msg = receive(sub);
    vec = [vec msg.Data];
    mean(vec)
    var(vec)
end