export KICAD6_3DMODEL_DIR=/usr/share/kicad/3dmodels/
mkdir -p export
kicad2step -o export/bwbots_beer_caddy.step bw_beer_caddy.kicad_pcb --subst-models -f --user-origin=135x79mm

