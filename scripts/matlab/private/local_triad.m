%% draw a local triad (coordinate frame) at the origin of the given transformation matrix T, with specified axis length L, line width lw, and line style ls
function local_triad(ax, T, L, lw, ls)
    % o = origin of the triad in 3D space, extracted from the translation component of the transformation matrix T
    o = T(1:3,4);

    % define the colors for the X, Y, and Z axes of the triad (red, green, blue)
    cols = {[0.85 0 0], [0 0.65 0], [0 0 0.9]};

    % loop through each axis (X, Y, Z) and plot a line representing the axis from the origin to the endpoint defined by the axis direction scaled by the length L
    for a = 1:3
        d = T(1:3, a);

        % compute the endpoint
        p1 = o + L * d;

        % plot a line from the origin to the endpoint of the axis in 3D space, using the specified color, line width, and line style
        plot3(ax, [o(1) p1(1)], [o(2) p1(2)], [o(3) p1(3)], ...
            'Color', cols{a}, 'LineWidth', lw, 'LineStyle', ls);

        % draw arrowhead (cone-like basic shape using line segments)
        headL = L * 0.15;  % arrow head length
        headW = L * 0.05;  % arrow head width
        if headL > 0
            % find two orthogonal vectors to form the base of the arrow head
            if abs(d(3)) < 0.9
                u = cross([0; 0; 1], d);
            else
                u = cross([1; 0; 0], d);
            end
            u = u / norm(u);
            v = cross(d, u);

            % base points of the arrow head
            pb = p1 - headL * d;
            p1b1 = pb + headW * u;
            p1b2 = pb - headW * u;
            p1b3 = pb + headW * v;
            p1b4 = pb - headW * v;

            % plot arrow head lines
            hX = [p1(1) p1b1(1) NaN p1(1) p1b2(1) NaN p1(1) p1b3(1) NaN p1(1) p1b4(1)];
            hY = [p1(2) p1b1(2) NaN p1(2) p1b2(2) NaN p1(2) p1b3(2) NaN p1(2) p1b4(2)];
            hZ = [p1(3) p1b1(3) NaN p1(3) p1b2(3) NaN p1(3) p1b3(3) NaN p1(3) p1b4(3)];

            line(ax, hX, hY, hZ, 'Color', cols{a}, 'LineWidth', lw, 'LineStyle', '-');
        end
    end
end
