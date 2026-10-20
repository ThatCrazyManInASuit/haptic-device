#ifndef ATOM_H
#define ATOM_H

#include "chai3d.h"
#include <mutex>
#include <queue>
#include <set>


class Atom : public chai3d::cShapeSphere { 
    private: 
        bool anchor; 
        bool current; 
        bool repeating; 
        bool selected; 
        chai3d::cVector3d velocity; 
        chai3d::cVector3d force; 
        chai3d::cVector3d prevForce; 
        chai3d::cVector3d prevPos; 
        std::vector<std::vector<std::vector<chai3d::cShapeSphere*>>> periodics; 
        
        // Separated line and comment safely
        std::queue<chai3d::cVector3d> positionBuffer; // a position buffer so physics calculator calculate ahead 
        // positionBuffer and prevPos are filled by the physics thread and drained by the
        // graphics thread, so every access to them has to hold this.
        mutable std::mutex bufferMutex;
        std::set<int> bondedAtoms; 
        
        chai3d::cShapeLine *velVector; // a rendered line that represents the atom's velocity 
        int atomicNumber; 
        chai3d::cColorf color; 
        void refreshMaterial(chai3d::cShapeSphere *sphere);

        /**
         * @brief Gets the latest buffered position. The caller must already hold bufferMutex.
         * @return the latest buffered position, or the rendered position if the buffer is empty
         */
        chai3d::cVector3d latestPosLocked() const;

    public:
        void setPeriodics(int x, int y, int z);

        /**
         * @brief A constructor for an atom
         * @param radius the radius of the atom. Should be based on the covalent radius
         * @param atomicNum the atomic number of the atom
         */
        Atom(double radius, int atomicNumber, chai3d::cWorld *world, chai3d::cTexture2dPtr texture);

        const std::vector<std::vector<std::vector<chai3d::cShapeSphere*>>>& getPeriodics() const;

        /**
         * @brief Returns if the atom is anchor
         * @return true if the atom is anchored, false otherwise
         */
        bool isAnchor() const;

        /**
         * @brief Sets if the atom should be anchored
         * @param newAnchor if the atom should be anchored
         */
        void setAnchor(bool newAnchor);

        /**
         * @brief Returns if the atom is repeating
         * @return true if the atom is repeating, false otherwise
         */
        bool isRepeating() const;

        /**
         * @brief Sets if the atom should be repeating
         * @param newRepeat if the atom should be repeating
         */
        void setRepeating(bool newRepeat);

        /**
         * @brief Returns if the atom is current
         * @return true if the atom is current, false otherwise
         */
        bool isCurrent() const;

        /**
         * @brief Sets if the atom should be current
         * @param newCurrent if the atom should be current
         */
        void setCurrent(bool newCurrent);

        /**
         * @brief Returns if the atom is selected
         * @return true if the atom is selected, false otherwise
         */
        bool isSelected() const;

        /**
         * @brief Sets if the atom should be selected
         * @param newSelected if the atom should be selected
         */
        void setSelected(bool newSelected);

        /**
         * @brief Gets the velocity of the atom
         * @return the velocity of the atom in world units. One world unit is 50 Å.
         */
        chai3d::cVector3d getVelocity() const;
        
        /**
         * @brief Sets the velocity of the atom in world units.
         * @param newVel the velocity of the atom in world units. One world unit is 50 Å.
         */
        void setVelocity(chai3d::cVector3d newVel);

        /**
         * @brief Gets the force applied to the atom
         * @return the force applied to the atom in eV/Å
         */
        chai3d::cVector3d getForce() const;

        /**
         * @brief Sets the force applied to the atom
         * @param newForce the force to apply to the atom in eV/Å
         */
        void setForce(chai3d::cVector3d newForce);

        /**
         * @brief Gets the force previous to the current applied force.
         * @return the force previous to the current applied force
         */
        chai3d::cVector3d getPrevForce() const;

        /**
         * @brief Gets the velocity vector of the atom as a rendered line
         * @return the velocity vector of the atom as a rendered line
         */
        chai3d::cShapeLine *getVelVector() const;

        /**
         * @brief Sets the rendered velocity vector of the atom
         * @param newVelVector The new rendered velocity vector of the atom
         */
        void setVelVector(chai3d::cShapeLine *newVelVector);

        /**
         * @brief Update the atom's rendered velocity vector
         */
        void updateForceVector();

        /**
         * @brief Sets the color of the atom
         * @param color The color to set the atom to
         */
        void setColor(chai3d::cColorf color);

        /**
         * @brief Gets the element color of the atom, ignoring any highlight (current, anchored,
         *        or repeating) currently painted over it
         * @return the element color of the atom
         */
        chai3d::cColorf getColor() const;

        /**
         * @brief Gets the atomic number of the atom
         * @return the atomic number of the atom
         */
        int getAtomicNumber() const;

        /**
         * @brief Sets the atomic number of the atom
         * @param num the atomic number of the atom to set to
         */
        void setAtomicNumber(int num);
        
        /**
         * @brief Gets the chemical symbol of the atom
         * @return the chemical symbol of the atom
         */
        std::string getElement() const;

        /**
         * @brief Gets the mass of the atom
         * @return the atomic mass of the atom in atomic mass units (amu)
         */
        double getMass() const;

        /**
         * @brief Gets the electronegativity of the atom
         * @return the electronegativity of the atom on the Pauling scale
         */
        double getEN() const;
        
        /**
         * @brief Sets the buffer position of the atom
         * @param pos the position to set the buffered pos to
         */
        void addBufferedPos(chai3d::cVector3d pos);

        /**
         * @brief Gets the buffer position of the atom
         * @return the buffered position of the atom
         */
        chai3d::cVector3d nextPos();

        /**
         * @brief Gets if the atom has a next buffered position
         * @return true if the atom has a next buffered position; false otherwise
         */
        bool hasNextPos() const;

        /**
         * @brief Gets the latest position added to the buffer
         * @return the latest position added to the buffer
         */
        chai3d::cVector3d getLatestPos() const;

        /**
         * @brief Gets the position most recently taken from the buffer
         * @return the position most recently taken from the buffer
         */
        chai3d::cVector3d getPrevPos() const;

        /**
         * @brief Returns the set of atoms bonded to the current atom, editable
         * @return the set of atoms bonded to the current atom, editable
         */
        std::set<int>& getBondedAtoms();
};

#endif  // ATOM_H
