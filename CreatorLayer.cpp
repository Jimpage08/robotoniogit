#include "CreatorLayer.h"
#include "Layer.h"
#include "AssetLayer.h"
#include "ClickLayer.h"
bool _fastcall CreatorLayer::hook(CCLayer* self) {
	bool result = CreatorLayer::init(self);
	auto director = CCDirector::sharedDirector();
	auto size = director->getWinSize();
	CCSprite* button = CCSprite::create("GJ_topBtn_001.png");
	button->setScale(0.85F);
	auto nextButton = gd::CCMenuItemSpriteExtra::create(button, self, menu_selector(Layer::callback));
	CCMenu* menu = reinterpret_cast<CCMenu*>(self->getChildren()->objectAtIndex(1));
	menu->addChild(nextButton);
	self->addChild(menu);
	CCSprite* assetbutton = CCSprite::create("GJ_assetBtn_001.png");
	assetbutton->setScale(0.85F);
	auto nextassetbutton = gd::CCMenuItemSpriteExtra::create(assetbutton, self, menu_selector(AssetLayer::callback));
	menu->addChild(nextassetbutton);
	CCSprite* clickbutton = CCSprite::create("GJ_clickBtn_001.png");
	clickbutton->setScale(0.85F);
	auto nextclickbutton = gd::CCMenuItemSpriteExtra::create(clickbutton, self, menu_selector(ClickLayer::callback));
	menu->addChild(nextclickbutton);
	auto createbutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(0);
	auto savedbutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(1);
	auto highscorebutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(2);
	auto challengebutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(3);
	auto dailybutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(4);
	auto weeklybutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(5);
	auto gauntletsbutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(10);
	auto featuredbutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(6);
	auto famebutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(7);
	auto mappacksbutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(8);
	auto searchbutton = (gd::CCMenuItemSpriteExtra*)menu->getChildren()->objectAtIndex(9);
	createbutton->setPosition(-180, 97);
	savedbutton->setPosition(-85, 97);
	highscorebutton->setPosition(10, 97);
	challengebutton->setPosition(105, 97);
	dailybutton->setPosition(-85, 0);
	weeklybutton->setPosition(10, 0);
	featuredbutton->setPosition(-180, -97);
	famebutton->setPosition(200, 97);
	mappacksbutton->setPosition(105, -97);
	searchbutton->setPosition(200, -97);
	gauntletsbutton->setPosition(200, 0);
	nextassetbutton->setPosition(-179, -2);
	nextButton->setPosition(12, -98);
	nextclickbutton->setPosition(107, -2);
	return result;
}