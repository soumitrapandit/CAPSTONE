%And then we need a control Indexing Function which takes in t, ti, ui and
%returns u
function u = control(t,ti,ui)
    tau = ui(sum(t>=ti));
    a = t-ui(end,:);
    u = [tau;a];
end
