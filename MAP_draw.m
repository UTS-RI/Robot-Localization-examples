function MAP_draw(LineData, color)
    hold on;
    for j = 1:size(LineData,1)
        plot(LineData(j,1:2),LineData(j,3:4), color)
    end
    hold off;
end