# lunar-lockout-search

A working solver for the puzzle game Lunar Lockout is implemented using Anytime Weighted A-star search. Different heuristics are designed to suplement the search: L-distance, Manhattan distance and Extended L-distance (heur_alternate).

## Game Description
Lunar Lockout is a puzzle game that is played on an NxN board. The game requires 'helper robots' to guide 'rovers' (also called 'xanadus') into an escape hatch that is located at the center of the board. The rules hold that pieces (i.e. helper bots and rovers) must move one at a time in a straight line; they may not move diagonally. Pieces also cannot move through one another. Moreover, each piece may only move in the direction of a second piece, and it must move until it collides with the second piece and comes to a stop. Moves may sometimes take pieces directly across the escape hatch or result in a robot blocking the escape hatch (i.e. occupying the same square as the escape hatch). A rover cannot exit the escape hatch unless in lands directly atop it. The game is over when all rovers have successfully made it through the escape hatch.
