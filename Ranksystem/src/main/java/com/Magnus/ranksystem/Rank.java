package com.Magnus.ranksystem;

import org.bukkit.ChatColor;

public enum Rank {
    OWNER(ChatColor.DARK_RED + "Owner"),
    ADMIN(ChatColor.RED + "Admin"),
    MEMBER(ChatColor.YELLOW + "Member"),
    GUEST(ChatColor.GRAY + "Guest");

    private String display;

    Rank(String display) {
        this.display = display;
    }

    public String getDisplay() { return display; }
}
